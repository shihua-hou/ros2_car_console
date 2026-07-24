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
    std::sort(pcd_files.begin(), pcd_files.end());
  }
  if (pcd_files.empty()) {
    RCLCPP_ERROR(logger, "在 %s 下没有找到 scans*.pcd (请先完成建图并Ctrl+C退出以保存PCD)", pcd_dir.c_str());
    rclcpp::shutdown();
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto &f : pcd_files) {
    pcl::PointCloud<pcl::PointXYZI> part;
    RCLCPP_INFO(logger, "读取点云: %s ...", f.c_str());
    if (pcl::io::loadPCDFile(f, part) < 0) {
      RCLCPP_WARN(logger, "读取失败, 跳过: %s", f.c_str());
      continue;
    }
    *cloud += part;
  }
  if (cloud->empty()) {
    RCLCPP_ERROR(logger, "点云为空 (请先完成建图并Ctrl+C退出以保存PCD)");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "共 %zu 个文件, 合计 %zu 点", pcd_files.size(), cloud->size());

  if (voxel > 1e-3) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel, voxel, voxel);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*filtered);
    cloud = filtered;
    RCLCPP_INFO(logger, "降采样后 %zu 点", cloud->size());
  }

  if (align_ground) {
    // 地面必然低于雷达(z<0), 只在低处点中拟合, 避免锁到天花板
    pcl::PointCloud<pcl::PointXYZI>::Ptr low(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3.0, 0.0);
    pass.filter(*low);

    bool aligned = false;
    if (low->size() > 1000) {
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setAxis(Eigen::Vector3f::UnitZ());
      seg.setEpsAngle(20.0 * M_PI / 180.0);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.05);
      seg.setMaxIterations(1000);
      seg.setInputCloud(low);
      seg.segment(*inliers, *coef);

      if (inliers->indices.size() > low->size() / 10) {
        Eigen::Vector3f n(coef->values[0], coef->values[1], coef->values[2]);
        float d = coef->values[3];
        if (n.z() < 0) { n = -n; d = -d; }
        n.normalize();
        const double lidar_height = d;  // 平面 n·p + d = 0, 原点(雷达)到地面距离
        // 旋转使地面法向对齐+Z, 再平移使地面落在 z=0
        Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f tf = Eigen::Translation3f(0, 0, d) * Eigen::Affine3f(q);
        pcl::transformPointCloud(*cloud, *cloud, tf);
        const double tilt_deg = std::acos(std::min(1.0f, n.z())) * 180.0 / M_PI;
        RCLCPP_INFO(logger, "地面校平完成: 拟合倾角 %.1f°, 雷达离地高度约 %.2fm "
                    "(可用于launch的lidar_z参数), 内点 %zu",
                    tilt_deg, lidar_height, inliers->indices.size());
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

  std::vector<int8_t> grid(width * height, -1);
  for (int i = 0; i < width * height; ++i) {
    if (occ_cnt[i] >= occ_pts)       grid[i] = 100;
    else if (free_cnt[i] >= free_pts) grid[i] = 0;
  }

  bool ok = save_map(map_dir, map_name, grid, width, height, resolution, min_x, min_y, logger);
  if (ok && !backup_dir.empty()) {
    save_map(backup_dir, map_name, grid, width, height, resolution, min_x, min_y, logger);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
