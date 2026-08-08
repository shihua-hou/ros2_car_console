/*
 * pcd2pgm: 将 FAST-LIO2 保存的 3D 点云地图 (PCD) 转换为 Nav2 可用的 2D 栅格地图 (PGM+YAML)
 *
 * 原理:
 *   1. 读取 PCD 点云 (坐标系为 FAST-LIO 的 camera_init)
 *   2. RANSAC 拟合地面并把点云校平, 使地面位于 z=0 (消除雷达安装倾斜/初始姿态误差,
 *      否则远处地面会翘进障碍物高度带, 地图上满地噪点)
 *   3. 取 [min_z, max_z] 高度带(相对地面)内的点投影为障碍物
 *   4. 取 min_z 以下的点(地面)投影为可通行区域, 其余为未知区域
 */
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

static bool save_map(const std::string &dir, const std::string &map_name,
                     const std::vector<int8_t> &grid, int width, int height,
                     double resolution, double origin_x, double origin_y,
                     rclcpp::Logger logger)
{
  const std::string pgm_path = dir + "/" + map_name + ".pgm";
  const std::string yaml_path = dir + "/" + map_name + ".yaml";

  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm.is_open()) {
    RCLCPP_ERROR(logger, "无法写入 %s", pgm_path.c_str());
    return false;
  }
  pgm << "P5\n" << width << " " << height << "\n255\n";
  // PGM 第一行对应地图最上方(y最大), 栅格按行倒序写入
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      int8_t v = grid[y * width + x];
      unsigned char pix;
      if (v == 100)      pix = 0x00;  // 占用: 黑
      else if (v == 0)   pix = 0xFE;  // 空闲: 白
      else               pix = 0xCD;  // 未知: 灰 (205)
      pgm.write(reinterpret_cast<char *>(&pix), 1);
    }
  }
  pgm.close();

  std::ofstream yaml(yaml_path);
  if (!yaml.is_open()) {
    RCLCPP_ERROR(logger, "无法写入 %s", yaml_path.c_str());
    return false;
  }
  yaml << "image: " << map_name << ".pgm\n"
       << "mode: trinary\n"
       << "resolution: " << resolution << "\n"
       << "origin: [" << origin_x << ", " << origin_y << ", 0]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.196\n";
  yaml.close();

  RCLCPP_INFO(logger, "地图已保存: %s (%dx%d, %.2fm/格)", yaml_path.c_str(), width, height, resolution);
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pcd2pgm");
  auto logger = node->get_logger();

  // pcd_file 非空则只读取该文件; 否则合并 pcd_dir 下所有 scans*.pcd 分段
  const std::string pcd_file = node->declare_parameter<std::string>("pcd_file", "");
  const std::string pcd_dir = node->declare_parameter<std::string>(
      "pcd_dir", "/home/cat/wheeltec_ros2/src/FAST_LIO/PCD");
  const std::string map_dir = node->declare_parameter<std::string>("map_dir", "/home/cat/maps");
  const std::string backup_dir = node->declare_parameter<std::string>("backup_dir", "");
  const std::string map_name = node->declare_parameter<std::string>("map_name", "WHEELTEC3D");
  const double resolution = node->declare_parameter<double>("resolution", 0.05);
  // 自动拟合地面并校平, 校平后地面在 z=0, min_z/max_z 均相对地面
  const bool align_ground = node->declare_parameter<bool>("align_ground", true);
  // 雷达安装角(弧度), 必须与 mapping.launch.py 里的一致 —— 点云所在的 camera_init 系
  // 是跟着雷达一起歪的, 拟合地面时要拿它算搜索轴。详见下面 align_ground 段的说明。
  const double lidar_pitch = node->declare_parameter<double>("lidar_pitch", 0.191);
  const double lidar_roll = node->declare_parameter<double>("lidar_roll", 0.006);
  // 障碍物高度带(相对地面): 默认取地面上方 0.10 ~ 0.60m (机器人会碰到的高度)
  const double min_z = node->declare_parameter<double>("min_z", 0.10);
  const double max_z = node->declare_parameter<double>("max_z", 0.60);
  // 低于 min_z 的点视为地面(标记空闲); 低于 ground_min_z 的杂散点忽略
  const double ground_min_z = node->declare_parameter<double>("ground_min_z", -0.5);
  // 单格内达到该点数才算障碍/地面: 障碍阈值高一些过滤噪点/动态行人残影,
  // 地面阈值低一些让可通行区域更连片
  const int occ_pts = node->declare_parameter<int>("occupied_min_points", 5);
  const int free_pts = node->declare_parameter<int>("free_min_points", 1);
  // 先对点云降采样, 防止大地图内存过大 (0=不降采样)
  const double voxel = node->declare_parameter<double>("voxel_size", 0.03);
  // 离群点裁剪: 保留 [outlier_pct, 100-outlier_pct] 分位数范围(再放宽1m)之内的点。
  // 0=不裁。详见下面裁剪那段的注释 —— 不裁的后果是静默的, 别关。
  const double outlier_pct = node->declare_parameter<double>("outlier_percentile", 0.1);
  // 分段抽稀: 最多读多少段。**默认 0=不限**, 因为已改成边读边降采样, 内存与分段数无关。
  // 保留这两个参数是给"只想快速看一眼"用的; 注意抽稀会显著减少自由区(地面点变稀)。
  const int max_files = node->declare_parameter<int>("max_files", 0);
  const int file_stride = node->declare_parameter<int>("file_stride", 0);
  // 雷达光心离地高度(米)。>0 时**直接用它定地面高度**, 只用RANSAC拟合地面法向做校平。
  // 为什么要有这个: 镜面地面场地里地面回波极少, RANSAC 拟合出来的"地面高度"很不可靠
  // (2026-08-06 水平安装时拟合出 0.443m, 真值 0.28m, 差 16cm, 而且残差只有13mm;
  // 倾装 11° 后地面回波变好, 拟合值 0.27 与真值只差 1cm —— 能不能信取决于倾没倾)。
  // 高度错 15cm 会让障碍带 [min_z,max_z] 整体平移, 矮障碍漏检、可通行区被误判。
  // 填 0 则退回原来的行为(用拟合值), 日志里仍会打印拟合值供对照。
  const double known_lidar_z = node->declare_parameter<double>("lidar_z", 0.28);
  // 判断"地面拟合到底可不可信"的门槛: 内点 / 低处候选点 的占比。
  // 镜面/抛光地面掠射角下几乎收不到地面回波, 这个占比只有个位数; 地毯等漫反射
  // 地面能到 30~50%。占比高就说明拟合值比卷尺值更可信 —— 见下面校平那段的守卫。
  const double fit_trust_ratio = node->declare_parameter<double>("ground_fit_trust_ratio", 0.25);

  std::vector<std::string> pcd_files;
  if (!pcd_file.empty()) {
    pcd_files.push_back(pcd_file);
  } else {
    std::error_code ec;
    for (const auto &e : std::filesystem::directory_iterator(pcd_dir, ec)) {
      const std::string name = e.path().filename().string();
      if (name.rfind("scans", 0) == 0 && e.path().extension() == ".pcd") {
        pcd_files.push_back(e.path().string());
      }
    }
    // 按分段序号排序: scans.pcd, scans_1.pcd, ... scans_286.pcd
    // 直接字典序会把 scans_10 排在 scans_2 前面, 抽稀时就不是均匀取样了
    auto seq_of = [](const std::string &path) {
      const size_t p1 = path.find_last_of('/');
      std::string nm = (p1 == std::string::npos) ? path : path.substr(p1 + 1);
      const size_t u = nm.find('_');
      if (u == std::string::npos) return -1;           // scans.pcd 排最前
      try { return std::stoi(nm.substr(u + 1)); } catch (...) { return 1 << 30; }
    };
    std::sort(pcd_files.begin(), pcd_files.end(),
              [&](const std::string &a, const std::string &b) {
                return seq_of(a) < seq_of(b);
              });
  }

  // ---------------- 长时间建图的分段抽稀 (2026-08-06 新增) ----------------
  // FAST-LIO 每 100 帧(约10秒)落一段, 每段约 16MB。**一次 47 分钟的建图 = 286 段 = 4.6GB**,
  // 全部读进内存需要 4.6GB, 而本机只有 3.8GB —— 必然 OOM, 而且磁盘也扛不住。
  // 相邻分段在空间上高度重叠(10秒里车最多走两三米, 而雷达能看十几米), 做 2D 地图时
  // 均匀抽取一部分完全够用。默认自动抽到 max_files 段以内。
  if (!pcd_files.empty()) {
    int stride = file_stride;
    if (stride <= 0) {
      stride = (max_files > 0 && (int)pcd_files.size() > max_files)
                   ? (int)std::ceil((double)pcd_files.size() / max_files) : 1;
    }
    if (stride > 1) {
      std::vector<std::string> picked;
      for (size_t i = 0; i < pcd_files.size(); i += stride) picked.push_back(pcd_files[i]);
      if (picked.back() != pcd_files.back()) picked.push_back(pcd_files.back());
      RCLCPP_WARN(logger,
                  "分段过多(%zu 段), 每 %d 段取 1 段 -> 实际读取 %zu 段。"
                  "相邻分段空间上高度重叠, 做2D地图不影响; 要全读请设 file_stride:=1"
                  "(注意内存: 每段约16MB)",
                  pcd_files.size(), stride, picked.size());
      pcd_files.swap(picked);
    }
  }
  if (pcd_files.empty()) {
    RCLCPP_ERROR(logger, "在 %s 下没有找到 scans*.pcd (请先完成建图并Ctrl+C退出以保存PCD)", pcd_dir.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // ---------------- 边读边降采样 (2026-08-06) ----------------
  // 为什么不能"全读进来再降采样": 一次 47 分钟的建图有 286 段、4.6GB, 而本机只有
  // 3.8GB 内存 —— 读到一半就 OOM。而且**抽稀分段是有代价的**: 障碍点密集, 丢几段
  // 不影响; 但自由区完全靠地面点判定, 镜面地面本来回波就稀, 丢 80% 的段等于丢 80%
  // 的地面观测(实测自由区 11.8% -> 3.3%)。所以要全读, 只能边读边压。
  //
  // 每读几个文件就体素降采样一次, 内存占用与分段数无关(只与地图体积有关, 实测稳定
  // 在 60 万点上下)。降采样前先按**每个文件自己的分位数**裁掉离群点 —— 否则少数
  // 野点会把包围盒撑大, VoxelGrid 的 int32 体素索引溢出后会**静默不降采样**
  // (PCL 只在 stderr 打一行 Leaf size is too small), 内存照样爆。
  auto robust_crop = [&](pcl::PointCloud<pcl::PointXYZI> &pc, double pct, float margin) {
    if (pc.size() < 1000 || pct <= 0.0) return;
    std::vector<float> v; v.reserve(pc.size());
    float lo[3], hi[3];
    for (int ax = 0; ax < 3; ++ax) {
      v.clear();
      for (const auto &p : pc.points) {
        const float c = (ax == 0) ? p.x : (ax == 1) ? p.y : p.z;
        if (std::isfinite(c)) v.push_back(c);
      }
      if (v.empty()) return;
      size_t k1 = (size_t)(pct / 100.0 * (v.size() - 1));
      size_t k2 = (size_t)((100.0 - pct) / 100.0 * (v.size() - 1));
      std::nth_element(v.begin(), v.begin() + k1, v.end()); lo[ax] = v[k1];
      std::nth_element(v.begin(), v.begin() + k2, v.end()); hi[ax] = v[k2];
    }
    pcl::PointCloud<pcl::PointXYZI> keep;
    keep.reserve(pc.size());
    for (const auto &p : pc.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.x < lo[0] - margin || p.x > hi[0] + margin ||
          p.y < lo[1] - margin || p.y > hi[1] + margin ||
          p.z < lo[2] - margin || p.z > hi[2] + margin) continue;
      keep.push_back(p);
    }
    pc.swap(keep);
  };
  auto shrink = [&](pcl::PointCloud<pcl::PointXYZI>::Ptr &c) {
    if (voxel <= 1e-3 || c->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(c);
    vg.setLeafSize(voxel, voxel, voxel);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*out);
    if (!out->empty()) c = out;      // 万一 VoxelGrid 失败(返回空), 保留原云
  };

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  size_t raw_total = 0;
  int nread = 0;
  for (const auto &f : pcd_files) {
    pcl::PointCloud<pcl::PointXYZI> part;
    if (pcl::io::loadPCDFile(f, part) < 0) {
      RCLCPP_WARN(logger, "读取失败, 跳过: %s", f.c_str());
      continue;
    }
    raw_total += part.size();
    robust_crop(part, outlier_pct, 1.0f);
    *cloud += part;
    if (++nread % 8 == 0) {
      const size_t before = cloud->size();
      shrink(cloud);
      RCLCPP_INFO(logger, "已读 %d/%zu 段, 累计原始 %zu 点, 降采样后 %zu 点",
                  nread, pcd_files.size(), raw_total, cloud->size());
      (void)before;
    }
  }
  shrink(cloud);
  RCLCPP_INFO(logger, "共 %d 个文件, 原始合计 %zu 点, 边读边降采样后 %zu 点",
              nread, raw_total, cloud->size());
  if (cloud->empty()) {
    RCLCPP_ERROR(logger, "点云为空 (请先完成建图并Ctrl+C退出以保存PCD)");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "共 %zu 个文件, 合计 %zu 点", pcd_files.size(), cloud->size());

  // ---------------- 离群点裁剪 (2026-08-06 新增, 必须在降采样之前) ----------------
  // 症状: 几十个离群点就能把包围盒撑到 150x173x54m(实际内容只有 23x19m), 后果有两个,
  // 而且都是**静默**的:
  //   ① VoxelGrid 用 int32 做体素索引, 盒子一大就溢出 —— PCL 只在 stderr 打一行
  //      "Leaf size is too small for the input dataset", 然后**原样返回不降采样**。
  //      日志里"降采样后 N 点"和输入一模一样就是这个。
  //   ② 地图按包围盒开尺寸 -> 960x813 格, 真实内容缩在角落, 未知率 97%。
  // 实测这批点云: 99.9% 的点在 17.8m 内、中位数 2.88m, 但最远的到 95m、z 到 49m。
  // 来源多半是镜面地面/玻璃的多路径反射, 以及 Livox 偶发的野点。
  // 用分位数而不是固定量程: 地图多大都自适应, 不会把真实的远处内容裁掉。
  if (outlier_pct > 0.0 && cloud->size() > 1000) {
    auto pct = [&](std::vector<float> &v, double q) {
      size_t k = (size_t)(q / 100.0 * (v.size() - 1));
      std::nth_element(v.begin(), v.begin() + k, v.end());
      return v[k];
    };
    std::vector<float> xs, ys, zs;
    xs.reserve(cloud->size()); ys.reserve(cloud->size()); zs.reserve(cloud->size());
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z);
    }
    const double lo = outlier_pct, hi = 100.0 - outlier_pct;
    std::vector<float> t;
    t = xs; const float x0 = pct(t, lo); t = xs; const float x1 = pct(t, hi);
    t = ys; const float y0 = pct(t, lo); t = ys; const float y1 = pct(t, hi);
    t = zs; const float z0 = pct(t, lo); t = zs; const float z1 = pct(t, hi);
    const float m = 1.0f;   // 分位数之外再留 1m 余量, 免得削掉边缘的墙
    pcl::PointCloud<pcl::PointXYZI>::Ptr kept(new pcl::PointCloud<pcl::PointXYZI>);
    kept->reserve(cloud->size());
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.x < x0 - m || p.x > x1 + m || p.y < y0 - m || p.y > y1 + m ||
          p.z < z0 - m || p.z > z1 + m) continue;
      kept->push_back(p);
    }
    const size_t dropped = cloud->size() - kept->size();
    RCLCPP_INFO(logger,
                "离群裁剪(%.2f%%分位+1m余量): 丢弃 %zu 点(%.3f%%), 包围盒 %.1fx%.1fx%.1f m",
                outlier_pct, dropped, 100.0 * dropped / std::max<size_t>(1, cloud->size()),
                x1 - x0 + 2 * m, y1 - y0 + 2 * m, z1 - z0 + 2 * m);
    cloud = kept;
  }


  if (align_ground) {
    // 【坐标系】点云是 FAST-LIO 的 camera_init 系, 而 camera_init **不是重力对齐的**:
    // IMU_Processing 初始化时 rot=单位阵、grav=-mean_acc, 也就是说世界系直接取了
    // 开机瞬间的 IMU 姿态。雷达怎么装, 这个系就怎么歪。
    // 2026-08-06 雷达曾下倾42°, 那时地面法向在 camera_init 系里离 +Z 有 42°,
    // 而原代码写死 setAxis(UnitZ)+20°容差 -> **一个内点都找不到, save_map 直接失败**。
    // 于是改成按 launch 传进来的安装角算出"地面法向的期望方向"当搜索轴。
    // 雷达已改回水平安装, 参数默认 0/0, 此时期望方向就是 +Z, 行为与原版完全一致;
    // 保留这两个参数是为了以后再倾装时不必改代码。
    const double lp = lidar_pitch, lr = lidar_roll;
    Eigen::Vector3f up(-std::sin(lp),
                       std::sin(lr) * std::cos(lp),
                       std::cos(lr) * std::cos(lp));
    up.normalize();
    RCLCPP_INFO(logger, "地面法向期望方向(按安装角 pitch=%.1f° roll=%.1f°): (%.3f, %.3f, %.3f)",
                lp * 180.0 / M_PI, lr * 180.0 / M_PI, up.x(), up.y(), up.z());

    // 地面必然低于雷达, 只在低处点中拟合, 避免锁到天花板。
    // "低"要沿重力方向量(投影到 up), 不能再用原始 z —— 系是歪的。
    pcl::PointCloud<pcl::PointXYZI>::Ptr low(new pcl::PointCloud<pcl::PointXYZI>);
    low->reserve(cloud->size() / 2);
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const float h = up.x() * p.x + up.y() * p.y + up.z() * p.z;
      if (h > -3.0f && h < 0.0f) low->push_back(p);
    }
    RCLCPP_INFO(logger, "低处候选点 %zu / %zu", low->size(), cloud->size());

    bool aligned = false;
    if (low->size() > 1000) {
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setAxis(up);
      seg.setEpsAngle(20.0 * M_PI / 180.0);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.05);
      seg.setMaxIterations(1000);
      seg.setInputCloud(low);
      seg.segment(*inliers, *coef);

      if (inliers->indices.size() > low->size() / 10) {
        Eigen::Vector3f n(coef->values[0], coef->values[1], coef->values[2]);
        float d = coef->values[3];
        // 让法向朝"上"。系是歪的, 不能再用 n.z()<0 判断
        if (n.dot(up) < 0) { n = -n; d = -d; }
        n.normalize();
        const double fitted_height = d;  // 平面 n·p + d = 0, 原点(雷达)到地面的拟合距离
        // 地面高度用哪个: 传了 lidar_z 就用实测值, 否则退回拟合值。
        // 镜面地面场地里拟合值很不可靠(见参数声明处的注释), 而**法向**受影响小得多,
        // 所以这里只保留拟合的法向做校平, 高度另用实测值。
        const double inlier_ratio =
            low->empty() ? 0.0 : (double)inliers->indices.size() / (double)low->size();
        bool use_known = known_lidar_z > 1e-3;
        // 【守卫, 2026-08-08 走廊地毯实测后加】
        // lidar_z 这个覆盖开关本来只为"镜面地面, 拟合值不可信"而设。可它是无条件生效的,
        // 于是在地面回波正常的场地(地毯)里, 一个过期的 lidar_z 会静默毁掉整张图:
        //   实测 拟合 0.18m vs lidar_z 0.28m -> 校平后地面正好落在 z=+0.10 = min_z,
        //   整条走廊的地面被顶进障碍带 [min_z,max_z], 地图上是满屏麻点, 车过不去。
        // 差值一旦逼近 min_z 就必然出这个事, 所以: 拟合可信(内点占比够) + 差值过半个
        // min_z -> 以拟合值为准, 并且必须吼出来, 不能再当成"正常的对照信息"。
        if (use_known && inlier_ratio >= fit_trust_ratio &&
            std::fabs(fitted_height - known_lidar_z) >= 0.5 * min_z) {
          RCLCPP_WARN(logger,
                      "⚠️ 地面拟合可信(内点占比 %.0f%% ≥ %.0f%%, 说明地面回波正常), "
                      "但拟合值 %.2fm 与 lidar_z=%.2fm 差 %.2fm ≥ min_z/2 —— "
                      "照用 lidar_z 会把整片地面顶进障碍带 [%.2f, %.2f], 地图会出满地噪点。"
                      "**已改用拟合值 %.2fm**。请核对 launch 里的 lidar_z 是否过期, "
                      "本场地地面回波正常的话直接传 lidar_z:=0 即可。",
                      100.0 * inlier_ratio, 100.0 * fit_trust_ratio, fitted_height,
                      known_lidar_z, std::fabs(fitted_height - known_lidar_z), min_z, max_z,
                      fitted_height);
          use_known = false;
        }
        const double lidar_height = use_known ? known_lidar_z : fitted_height;
        // 旋转使地面法向对齐+Z, 再平移使地面落在 z=0
        Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f tf = Eigen::Translation3f(0, 0, (float)lidar_height) * Eigen::Affine3f(q);
        pcl::transformPointCloud(*cloud, *cloud, tf);
        // 实测地面法向与"按安装角推出的期望法向"的夹角。这不再是"雷达装歪了多少"
        // (雷达就是故意装歪42°的), 而是**安装角标定的残差**: >3° 说明 launch 里的
        // lidar_pitch/lidar_roll 与实际不符, 该重标了。
        const double resid_deg =
            std::acos(std::min(1.0f, std::max(-1.0f, n.dot(up)))) * 180.0 / M_PI;
        if (use_known) {
          // 拟合值只作对照。差多少才算正常, 取决于雷达倾没倾:
          //   水平安装 + 镜面地面 -> 地面几乎无回波, 拟合值可以差到 15~20cm, 属正常, 别管它
          //   倾装 10° 以上       -> 地面回波够了, 拟合值应当能对上; 差超过 5cm 就该查
          //                          (要么 lidar_z 填错了, 要么建图时 z 漂了)
          const double diff = std::fabs(fitted_height - lidar_height);
          const char *verdict =
              (diff < 0.05) ? "两者吻合"
                            : "⚠️ 两者差得多: 雷达若已倾装>10°, 说明 lidar_z 可能填错或建图时z漂;"
                              " 若是水平安装则属正常(镜面地面拟合不可信)";
          RCLCPP_INFO(logger,
                      "地面校平完成: 与期望法向偏差 %.1f°(>3°说明安装角标定不准), "
                      "地面高度用 lidar_z=%.2fm (点云拟合值 %.2fm, 差 %.2fm — %s), "
                      "内点 %zu(占低处点 %.0f%%)",
                      resid_deg, lidar_height, fitted_height, diff, verdict,
                      inliers->indices.size(), 100.0 * inlier_ratio);
        } else {
          RCLCPP_INFO(logger, "地面校平完成: 与期望法向偏差 %.1f°(>3°说明安装角标定不准), "
                      "雷达离地高度约 %.2fm (用的是点云拟合值), "
                      "内点 %zu(占低处点 %.0f%% —— 占比低=地面回波少, 拟合值不可信, "
                      "该场地应改传实测 lidar_z)",
                      resid_deg, lidar_height, inliers->indices.size(), 100.0 * inlier_ratio);
        }
        aligned = true;
      }
    }
    if (!aligned) {
      RCLCPP_WARN(logger, "地面拟合失败(低处点不足), 跳过校平; min_z/max_z 将以雷达高度为原点, 请自行调整");
    }
  }

  // 计算高度带内点的XY范围
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  for (const auto &p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.z < ground_min_z || p.z > max_z) continue;
    min_x = std::min(min_x, (double)p.x); max_x = std::max(max_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y); max_y = std::max(max_y, (double)p.y);
  }
  if (min_x > max_x) {
    RCLCPP_ERROR(logger, "高度带 [%.2f, %.2f] 内没有点, 请检查 min_z/max_z 与雷达安装高度", ground_min_z, max_z);
    rclcpp::shutdown();
    return 1;
  }
  // 地图边缘留 1m 余量
  min_x -= 1.0; min_y -= 1.0; max_x += 1.0; max_y += 1.0;

  const int width = (int)std::ceil((max_x - min_x) / resolution);
  const int height = (int)std::ceil((max_y - min_y) / resolution);
  if ((long)width * height > 400L * 1000 * 1000) {
    RCLCPP_ERROR(logger, "地图尺寸过大 %dx%d, 请增大 resolution", width, height);
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "栅格尺寸 %dx%d, 原点(%.2f, %.2f)", width, height, min_x, min_y);

  std::vector<uint16_t> occ_cnt(width * height, 0), free_cnt(width * height, 0);
  for (const auto &p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.z < ground_min_z || p.z > max_z) continue;
    const int gx = (int)((p.x - min_x) / resolution);
    const int gy = (int)((p.y - min_y) / resolution);
    if (gx < 0 || gx >= width || gy < 0 || gy >= height) continue;
    const int idx = gy * width + gx;
    if (p.z >= min_z) {
      if (occ_cnt[idx] < 65535) occ_cnt[idx]++;
    } else {
      if (free_cnt[idx] < 65535) free_cnt[idx]++;
    }
  }

  // ---- 自由区连片化 ----
  // 地面点在光滑/镜面地板上非常稀疏(掠射角下激光被镜面反射走, 收不到回波),
  // 直接按"格子里有地面点才算自由"会得到一片麻点, 中间全是未知空洞。
  // nav2 的 NavFn 默认 allow_unknown:false, 规划器过不去这些洞, 表现就是
  // "地图看着有房间, 但下目标点车不走"。
  // 2026-08-05 实测: 抛光地面的房间只有 3.9% 自由区、94.4% 未知。
  // 两步补救(只动自由区, 障碍永远优先, 不会把墙填掉):
  //   ① 形态学闭运算(先膨胀后腐蚀): 把麻点连成片、补掉小洞
  //   ② 填充封闭空洞: 被自由区/障碍完全围住、从地图边界走不到的未知格 -> 自由
  const int close_r = node->declare_parameter<int>("free_close_radius", 2);
  const bool fill_enclosed = node->declare_parameter<bool>("fill_enclosed", true);

  std::vector<uint8_t> freem(width * height, 0), occm(width * height, 0);
  for (int i = 0; i < width * height; ++i) {
    occm[i] = (occ_cnt[i] >= occ_pts) ? 1 : 0;
    freem[i] = (free_cnt[i] >= free_pts) ? 1 : 0;
  }
  const size_t free_raw = std::count(freem.begin(), freem.end(), (uint8_t)1);

  auto morph = [&](const std::vector<uint8_t> &src, int r, bool dilate) {
    std::vector<uint8_t> dst(src.size(), dilate ? 0 : 1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        uint8_t v = dilate ? 0 : 1;
        for (int dy = -r; dy <= r && (dilate ? !v : v); ++dy) {
          for (int dx = -r; dx <= r && (dilate ? !v : v); ++dx) {
            const int nx = x + dx, ny = y + dy;
            // 越界按"非自由"算: 膨胀时不贡献, 腐蚀时会削掉边缘(闭运算整体抵消)
            const uint8_t s = (nx < 0 || nx >= width || ny < 0 || ny >= height)
                              ? 0 : src[ny * width + nx];
            if (dilate) { if (s) v = 1; }
            else        { if (!s) v = 0; }
          }
        }
        dst[y * width + x] = v;
      }
    }
    return dst;
  };

  if (close_r > 0) {
    freem = morph(morph(freem, close_r, true), close_r, false);
  }
  const size_t free_closed = std::count(freem.begin(), freem.end(), (uint8_t)1);

  size_t filled = 0;
  if (fill_enclosed) {
    // 从地图四边泛洪, 标出"能从外界走到"的非自由非障碍格; 走不到的就是内部空洞
    std::vector<uint8_t> reach(width * height, 0);
    std::vector<int> stack;
    auto push = [&](int x, int y) {
      if (x < 0 || x >= width || y < 0 || y >= height) return;
      const int i = y * width + x;
      if (reach[i] || freem[i] || occm[i]) return;
      reach[i] = 1; stack.push_back(i);
    };
    for (int x = 0; x < width; ++x) { push(x, 0); push(x, height - 1); }
    for (int y = 0; y < height; ++y) { push(0, y); push(width - 1, y); }
    while (!stack.empty()) {
      const int i = stack.back(); stack.pop_back();
      const int x = i % width, y = i / width;
      push(x + 1, y); push(x - 1, y); push(x, y + 1); push(x, y - 1);
    }
    for (int i = 0; i < width * height; ++i) {
      if (!freem[i] && !occm[i] && !reach[i]) { freem[i] = 1; ++filled; }
    }
  }

  std::vector<int8_t> grid(width * height, -1);
  for (int i = 0; i < width * height; ++i) {
    if (occm[i])       grid[i] = 100;   // 障碍优先, 连片化不会吃掉墙
    else if (freem[i]) grid[i] = 0;
  }
  const size_t tot = (size_t)width * height;
  RCLCPP_INFO(logger,
    "自由区: 原始 %zu 格(%.1f%%) -> 闭运算(r=%d) %zu 格(%.1f%%) -> 填内部空洞 +%zu 格, "
    "最终自由 %.1f%% / 障碍 %.1f%% / 未知 %.1f%%",
    free_raw, 100.0 * free_raw / tot, close_r, free_closed, 100.0 * free_closed / tot, filled,
    100.0 * std::count(grid.begin(), grid.end(), (int8_t)0) / tot,
    100.0 * std::count(grid.begin(), grid.end(), (int8_t)100) / tot,
    100.0 * std::count(grid.begin(), grid.end(), (int8_t)-1) / tot);

  bool ok = save_map(map_dir, map_name, grid, width, height, resolution, min_x, min_y, logger);
  if (ok && !backup_dir.empty()) {
    save_map(backup_dir, map_name, grid, width, height, resolution, min_x, min_y, logger);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
