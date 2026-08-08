// depth_obstacle_filter —— Astra 深度图 → 导航用障碍点云
//
// 为什么不用驱动自带的 /camera/depth/points:
//   驱动发的是 640x480 稠密 XYZ 点云(3.7MB/帧 @18Hz ≈ 66MB/s), 在 4GB 内存的
//   鲁班猫4 上光是 DDS 搬运就很吃 CPU, 而 nav2 costmap 真正需要的只是几百个
//   "地面以上、车能撞到" 的点。本节点直接订阅深度图(614KB/帧), 按 stride 抽样
//   反投影, 在 base_footprint 系里做高度带过滤 + 体素去重, 输出通常几百点。
//
// 输出坐标系刻意保持相机光心系(深度图的 frame_id):
//   nav2 ObservationBuffer 用观测点云的 frame 原点作为射线起点做 raytrace 清除,
//   若改发 base_footprint 系, 清除射线就会从车中心而不是相机发出, 清错格子。
//   过滤判据(高度/距离)仍在 base_footprint 系里算, 只是发出去的是原始相机系坐标。
//
// 高度带过滤是防"地面被当成障碍"的关键: 相机有 1~2° 安装/标定倾角时, 3m 外的
// 地面点表观高度就能抬到 5~10cm, min_z 太低会在车前方刷出一圈幻影障碍
// (和 pointcloud_to_laserscan 的 min_height 是同一类坑)。

#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

// 体素键: 把 base 系坐标量化到 voxel 网格, 用于去重。范围有限(<=10m), 直接
// 打包成 64 位整数即可, 不需要哈希组合。
inline int64_t voxelKey(double x, double y, double z, double voxel)
{
  const int64_t ix = static_cast<int64_t>(std::floor(x / voxel)) + 4096;
  const int64_t iy = static_cast<int64_t>(std::floor(y / voxel)) + 4096;
  const int64_t iz = static_cast<int64_t>(std::floor(z / voxel)) + 4096;
  return (ix << 40) | (iy << 20) | iz;
}

}  // namespace

class DepthObstacleFilter : public rclcpp::Node
{
public:
  DepthObstacleFilter()
  : Node("depth_obstacle_filter")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    stride_ = declare_parameter<int>("stride", 4);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 4.0);
    min_z_ = declare_parameter<double>("min_z", 0.06);
    max_z_ = declare_parameter<double>("max_z", 1.20);
    voxel_ = declare_parameter<double>("voxel_size", 0.05);
    max_rate_ = declare_parameter<double>("max_rate", 10.0);
    max_points_ = declare_parameter<int>("max_points", 4000);

    if (stride_ < 1) {stride_ = 1;}
    if (voxel_ < 0.01) {voxel_ = 0.01;}

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "depth/camera_info", sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        fx_ = msg->k[0]; fy_ = msg->k[4];
        cx_ = msg->k[2]; cy_ = msg->k[5];
        has_info_ = (fx_ > 1.0 && fy_ > 1.0);
      });
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
      "depth/image_raw", sensor_qos,
      std::bind(&DepthObstacleFilter::onDepth, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "camera/obstacle_points", sensor_qos);

    RCLCPP_INFO(get_logger(),
      "depth_obstacle_filter: target=%s stride=%d 高度带[%.2f, %.2f]m "
      "距离[%.2f, %.2f]m voxel=%.2fm 上限%.1fHz",
      target_frame_.c_str(), stride_, min_z_, max_z_,
      min_range_, max_range_, voxel_, max_rate_);
  }

private:
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "还没收到 depth/camera_info, 暂不处理深度图");
      return;
    }
    // 限频: 相机 30Hz, costmap 只需要 5~10Hz, 多出来的帧直接丢, 省 CPU
    const rclcpp::Time now = get_clock()->now();
    if (max_rate_ > 0.0 && last_pub_.nanoseconds() > 0 &&
      (now - last_pub_).seconds() < 1.0 / max_rate_)
    {
      return;
    }

    const bool is_mm = (msg->encoding == "16UC1" || msg->encoding == "mono16");
    const bool is_m = (msg->encoding == "32FC1");
    if (!is_mm && !is_m) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "不支持的深度图编码: %s", msg->encoding.c_str());
      return;
    }

    // 相机光心系 -> base_footprint。用最新可用的TF(相机相对车体是静态的,
    // 不必严格对齐时间戳; 严格对齐反而会因深度图时间戳偏差频繁抛异常)
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                       tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF %s <- %s 不可用: %s", target_frame_.c_str(),
        msg->header.frame_id.c_str(), e.what());
      return;
    }

    // 四元数展开成旋转矩阵(只为了避免再拉一个 tf2_geometry_msgs 依赖)
    const double qx = tf.transform.rotation.x, qy = tf.transform.rotation.y;
    const double qz = tf.transform.rotation.z, qw = tf.transform.rotation.w;
    const double r00 = 1 - 2 * (qy * qy + qz * qz), r01 = 2 * (qx * qy - qz * qw),
      r02 = 2 * (qx * qz + qy * qw);
    const double r10 = 2 * (qx * qy + qz * qw), r11 = 1 - 2 * (qx * qx + qz * qz),
      r12 = 2 * (qy * qz - qx * qw);
    const double r20 = 2 * (qx * qz - qy * qw), r21 = 2 * (qy * qz + qx * qw),
      r22 = 1 - 2 * (qx * qx + qy * qy);
    const double tx = tf.transform.translation.x, ty = tf.transform.translation.y,
      tz = tf.transform.translation.z;

    const int w = static_cast<int>(msg->width), h = static_cast<int>(msg->height);
    const uint8_t * base = msg->data.data();

    keys_.clear();
    pts_.clear();
    for (int v = 0; v < h; v += stride_) {
      const uint8_t * row = base + static_cast<size_t>(v) * msg->step;
      for (int u = 0; u < w; u += stride_) {
        double d;
        if (is_mm) {
          uint16_t raw;
          std::memcpy(&raw, row + static_cast<size_t>(u) * 2, sizeof(raw));
          if (raw == 0) {continue;}
          d = raw * 0.001;
        } else {
          float raw;
          std::memcpy(&raw, row + static_cast<size_t>(u) * 4, sizeof(raw));
          if (!std::isfinite(raw) || raw <= 0.0f) {continue;}
          d = raw;
        }
        if (d < min_range_ || d > max_range_) {continue;}

        // 光心系: x右 y下 z前
        const double px = (u - cx_) * d / fx_;
        const double py = (v - cy_) * d / fy_;
        const double pz = d;

        const double bx = r00 * px + r01 * py + r02 * pz + tx;
        const double by = r10 * px + r11 * py + r12 * pz + ty;
        const double bz = r20 * px + r21 * py + r22 * pz + tz;

        // 地面以下/以上和车顶以上的点直接丢: 地面点是幻影障碍的主要来源
        if (bz < min_z_ || bz > max_z_) {continue;}
        if (std::hypot(bx, by) > max_range_) {continue;}

        if (!keys_.insert(voxelKey(bx, by, bz, voxel_)).second) {continue;}
        pts_.push_back({static_cast<float>(px), static_cast<float>(py),
                        static_cast<float>(pz)});
        if (static_cast<int>(pts_.size()) >= max_points_) {break;}
      }
      if (static_cast<int>(pts_.size()) >= max_points_) {break;}
    }

    // 即使一个点都没有也要发: costmap 侧靠"观测过期"判断传感器是否还活着
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;          // frame 保持相机光心系, 见文件头说明
    out.height = 1;
    out.width = static_cast<uint32_t>(pts_.size());
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts_.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(out, "x"), it_y(out, "y"),
      it_z(out, "z");
    for (const auto & p : pts_) {
      *it_x = p[0]; *it_y = p[1]; *it_z = p[2];
      ++it_x; ++it_y; ++it_z;
    }
    pub_->publish(out);
    last_pub_ = now;

    if ((++frame_count_ % 100) == 0) {
      RCLCPP_INFO(get_logger(), "相机障碍点云: %zu 点/帧", pts_.size());
    }
  }

  std::string target_frame_;
  int stride_{4};
  double min_range_{0.35}, max_range_{4.0};
  double min_z_{0.06}, max_z_{1.2};
  double voxel_{0.05}, max_rate_{10.0};
  int max_points_{4000};

  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool has_info_{false};
  rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
  uint64_t frame_count_{0};

  std::unordered_set<int64_t> keys_;
  std::vector<std::array<float, 3>> pts_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
