/*
 * auto_relocalize: 基于激光扫描匹配的自动重定位
 *
 * 原理: 对 2D 栅格地图的障碍物做距离变换得到似然场, 把当前 /scan 的激光末端点
 * 在 (x, y, yaw) 三维空间做粗到精的全局搜索打分, 取最优位姿发布到 /initialpose,
 * 由 AMCL 接管后续跟踪定位。
 *
 * 功能:
 *   1. 启动自动重定位: 导航启动后无需手动 "2D Pose Estimate"
 *   2. 手动触发: ros2 service call /relocalize std_srvs/srv/Trigger
 *   3. 绑架检测看门狗(默认开): 周期用当前激光给 AMCL 位姿打分,
 *      分数持续过低(小车被抱走/定位丢失)时自动触发全局重定位
 */
#include <cmath>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Candidate {
  double x = 0, y = 0, yaw = 0;
  double score = -1.0;
};

class AutoRelocalize : public rclcpp::Node {
public:
  AutoRelocalize() : Node("auto_relocalize") {
    auto_on_startup_ = declare_parameter<bool>("auto_on_startup", true);
    accept_score_ = declare_parameter<double>("accept_score", 0.55);
    sigma_ = declare_parameter<double>("sigma", 0.25);            // 似然场高斯宽度 m
    coarse_step_ = declare_parameter<double>("coarse_step", 0.2); // 粗搜位置步长 m
    coarse_yaw_step_ = declare_parameter<double>("coarse_yaw_step_deg", 10.0) * M_PI / 180.0;
    max_beam_range_ = declare_parameter<double>("max_beam_range", 15.0);
    min_clearance_ = declare_parameter<double>("min_clearance", 0.12); // 候选位置离障碍最小距离
    watchdog_en_ = declare_parameter<bool>("watchdog_en", true);
    // 看门狗用独立的窄高斯(σ=watchdog_sigma)严格打分: 宽高斯(sigma=0.25)是给
    // 全局搜索用的, 在障碍密集地图上错误位姿也能蒙到0.7分, 无法区分对错;
    // 窄高斯下激光点偏20cm几乎0分 → 正确位姿0.6~0.8, 被搬动后0.1~0.2。
    watchdog_sigma_ = declare_parameter<double>("watchdog_sigma", 0.08);
    watchdog_score_ = declare_parameter<double>("watchdog_score", 0.4);
    watchdog_count_ = declare_parameter<int>("watchdog_count", 2);
    num_threads_ = std::max(1, (int)declare_parameter<int>("num_threads", 4));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onMap(msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(scan_mutex_);
          scan_ = msg;
        });
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    srv_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          Candidate c;
          bool ok = relocalize(c, true);
          res->success = ok;
          res->message = ok ? "重定位成功 score=" + std::to_string(c.score)
                            : "重定位失败(匹配分过低) best=" + std::to_string(c.score);
          if (ok && pose_pub_->get_subscription_count() == 0) {
            res->message += " [警告: /initialpose 无订阅者, AMCL可能未激活]";
          }
        });
    // 看门狗定时器/TF 无条件创建, 是否真正打分重定位由运行期的 watchdog_en_ 决定
    // (这样 webapp 可以在"里程计模式/自动重定位模式"之间实时切换, 无需重启导航)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(2),
                                        [this]() { watchdogCheck(); });
    // 允许运行期通过 SetParameters 改看门狗行为 (里程计模式 = watchdog_en:false)
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &ps) {
          rcl_interfaces::msg::SetParametersResult r; r.successful = true;
          for (const auto &p : ps) {
            if (p.get_name() == "watchdog_en") watchdog_en_ = p.as_bool();
            else if (p.get_name() == "watchdog_score") watchdog_score_ = p.as_double();
            else if (p.get_name() == "watchdog_count") watchdog_count_ = (int)p.as_int();
          }
          RCLCPP_INFO(get_logger(), "重定位模式更新: 自动重定位看门狗=%s (score<%.2f 连续%d次)",
                      watchdog_en_ ? "开" : "关", watchdog_score_, watchdog_count_);
          return r;
        });
    startup_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
      if (!auto_on_startup_ || startup_done_) return;
      if (!field_ready_ || !haveScan()) return;
      // 等 AMCL 的 initialpose 订阅就位再发, 否则位姿发了也没人收
      if (pose_pub_->get_subscription_count() == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "等待 AMCL 订阅 /initialpose ...");
        return;
      }
      Candidate c;
      if (relocalize(c, false)) {
        startup_done_ = true;
        startup_timer_->cancel();
      } else if (++startup_tries_ >= 5) {
        RCLCPP_ERROR(get_logger(),
            "启动自动重定位失败(尝试5次, 最高分 %.2f), 请在rviz用 2D Pose Estimate 手动定位, "
            "或移动小车到特征更多的位置后调用 /relocalize 服务", c.score);
        startup_done_ = true;
        startup_timer_->cancel();
      }
    });
    RCLCPP_INFO(get_logger(), "自动重定位节点就绪 (启动自动定位: %s, 看门狗: %s)",
                auto_on_startup_ ? "开" : "关", watchdog_en_ ? "开" : "关");
  }

private:
  // ---------- 绑架检测看门狗 ----------
  // 用当前激光给 AMCL 位姿(TF map->base_footprint)打分:
  // 小车被抱走后 AMCL 仍自信地输出旧位姿, 但激光和地图对不上, 分数会骤降
  void watchdogCheck() {
    if (!watchdog_en_) return;   // 里程计模式: 不自动重定位, 侧重里程计(手动重定位仍可用)
    if (!field_ready_ || !haveScan() || !startup_done_) return;
    if ((now() - last_reloc_time_).seconds() < 15.0) return;  // 冷却期
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;  // AMCL尚未输出定位
    }
    const auto &q = tf.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // 抽样数原为120, 是按MID360转换出的稠密/scan(有效率接近100%)调的;
    // N10Plus等有效点率低(比如约10%)的2D激光同样的抽样数会大量落空,
    // 提高到600留足余量, 对高密度的MID360只是多算点、可忽略的开销
    const auto pts = beams(600);
    if (pts.size() < 30) return;
    const double sc = scorePoseField(pts, tf.transform.translation.x,
                                     tf.transform.translation.y,
                                     std::cos(yaw), std::sin(yaw), score_tight_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                         "定位健康度(严格分): %.2f (低于%.2f连续%d次触发重定位)",
                         sc, watchdog_score_, watchdog_count_);
    if (sc < watchdog_score_) {
      if (++low_score_cnt_ >= watchdog_count_) {
        RCLCPP_WARN(get_logger(),
            "当前位姿匹配分连续过低(%.2f < %.2f), 判定定位丢失(小车可能被移动), 触发全局重定位",
            sc, watchdog_score_);
        low_score_cnt_ = 0;
        Candidate c;
        relocalize(c, false);
      }
    } else {
      low_score_cnt_ = 0;
    }
  }

  // ---------- 地图与似然场 ----------
  void onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    map_ = msg;
    const int w = msg->info.width, h = msg->info.height;
    const float res = msg->info.resolution;
    // 距离变换 (两遍 chamfer)
    const float INF = 1e9f;
    dist_.assign((size_t)w * h, INF);
    for (int i = 0; i < w * h; ++i)
      if (msg->data[i] >= 65) dist_[i] = 0.0f;
    const float s = res, diag = res * 1.41421356f;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        float &d = dist_[(size_t)y * w + x];
        if (x > 0) d = std::min(d, dist_[(size_t)y * w + x - 1] + s);
        if (y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x] + s);
        if (x > 0 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x - 1] + diag);
        if (x < w - 1 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x + 1] + diag);
      }
    for (int y = h - 1; y >= 0; --y)
      for (int x = w - 1; x >= 0; --x) {
        float &d = dist_[(size_t)y * w + x];
        if (x < w - 1) d = std::min(d, dist_[(size_t)y * w + x + 1] + s);
        if (y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x] + s);
        if (x < w - 1 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x + 1] + diag);
        if (x > 0 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x - 1] + diag);
      }
    // 似然场: 宽高斯给全局搜索, 窄高斯给看门狗健康检查
    score_.resize(dist_.size());
    score_tight_.resize(dist_.size());
    const float inv2s2 = 1.0f / (2.0f * sigma_ * sigma_);
    const float inv2s2t = 1.0f / (2.0f * watchdog_sigma_ * watchdog_sigma_);
    for (size_t i = 0; i < dist_.size(); ++i) {
      score_[i] = std::exp(-dist_[i] * dist_[i] * inv2s2);
      score_tight_[i] = std::exp(-dist_[i] * dist_[i] * inv2s2t);
    }
    field_ready_ = true;
    RCLCPP_INFO(get_logger(), "似然场就绪 %dx%d", w, h);
  }

  bool haveScan() {
    std::lock_guard<std::mutex> lk(scan_mutex_);
    return scan_ != nullptr;
  }

  // 从scan提取激光末端点(载体坐标系), 均匀抽取 n 束
  std::vector<std::pair<float, float>> beams(int n) {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      scan = scan_;
    }
    std::vector<std::pair<float, float>> pts;
    if (!scan) return pts;
    const int total = scan->ranges.size();
    const int stride = std::max(1, total / n);
    for (int i = 0; i < total; i += stride) {
      const float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > max_beam_range_) continue;
      const float a = scan->angle_min + i * scan->angle_increment;
      pts.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    return pts;
  }

  // 单个位姿打分: 激光末端点落在似然场上的平均分
  inline double scorePoseField(const std::vector<std::pair<float, float>> &pts,
                               double x, double y, double c, double s,
                               const std::vector<float> &field) const {
    const auto &info = map_->info;
    const int w = info.width, h = info.height;
    double sum = 0;
    for (const auto &p : pts) {
      const double wx = x + c * p.first - s * p.second;
      const double wy = y + s * p.first + c * p.second;
      const int gx = (int)((wx - info.origin.position.x) / info.resolution);
      const int gy = (int)((wy - info.origin.position.y) / info.resolution);
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;  // 图外记0分
      sum += field[(size_t)gy * w + gx];
    }
    return pts.empty() ? 0.0 : sum / pts.size();
  }

  inline double scorePose(const std::vector<std::pair<float, float>> &pts,
                          double x, double y, double c, double s) const {
    return scorePoseField(pts, x, y, c, s, score_);
  }

  // 在给定候选位置集合 × 角度集合中并行搜索最优
  Candidate search(const std::vector<std::pair<double, double>> &positions,
                   const std::vector<double> &yaws,
                   const std::vector<std::pair<float, float>> &pts) {
    std::vector<Candidate> best(num_threads_);
    std::vector<std::thread> workers;
    std::atomic<size_t> next{0};
    for (int t = 0; t < num_threads_; ++t) {
      workers.emplace_back([&, t]() {
        size_t i;
        while ((i = next.fetch_add(1)) < positions.size()) {
          const auto &pos = positions[i];
          for (double yaw : yaws) {
            const double sc = scorePose(pts, pos.first, pos.second,
                                        std::cos(yaw), std::sin(yaw));
            if (sc > best[t].score) best[t] = {pos.first, pos.second, yaw, sc};
          }
        }
      });
    }
    for (auto &w : workers) w.join();
    Candidate b;
    for (const auto &c : best)
      if (c.score > b.score) b = c;
    return b;
  }

  bool relocalize(Candidate &result, bool from_service) {
    if (!field_ready_ || !haveScan()) {
      RCLCPP_WARN(get_logger(), "地图或激光数据未就绪, 无法重定位");
      return false;
    }
    const auto t0 = now();
    const auto &info = map_->info;
    const int w = info.width, h = info.height;

    // 粗搜: 所有离障碍够远的格子(含未知区), 按 coarse_step 抽样
    const int stride = std::max(1, (int)(coarse_step_ / info.resolution));
    std::vector<std::pair<double, double>> coarse_pos;
    for (int gy = 0; gy < h; gy += stride)
      for (int gx = 0; gx < w; gx += stride) {
        if (dist_[(size_t)gy * w + gx] < min_clearance_) continue;
        coarse_pos.emplace_back(info.origin.position.x + (gx + 0.5) * info.resolution,
                                info.origin.position.y + (gy + 0.5) * info.resolution);
      }
    std::vector<double> coarse_yaws;
    for (double a = -M_PI; a < M_PI; a += coarse_yaw_step_) coarse_yaws.push_back(a);
    auto pts_coarse = beams(400);   // 原60, 理由同上(watchdogCheck里的注释)
    if (pts_coarse.size() < 20) {
      RCLCPP_WARN(get_logger(), "有效激光束过少(%zu), 无法重定位", pts_coarse.size());
      return false;
    }
    Candidate best = search(coarse_pos, coarse_yaws, pts_coarse);

    // 精搜: 最优附近 ±0.3m/0.05m 步长, ±12°/2° 步长, 更多激光束
    std::vector<std::pair<double, double>> fine_pos;
    for (double dx = -0.3; dx <= 0.3; dx += 0.05)
      for (double dy = -0.3; dy <= 0.3; dy += 0.05)
        fine_pos.emplace_back(best.x + dx, best.y + dy);
    std::vector<double> fine_yaws;
    for (double da = -12 * M_PI / 180; da <= 12 * M_PI / 180; da += 2 * M_PI / 180)
      fine_yaws.push_back(best.yaw + da);
    auto pts_fine = beams(900);   // 原180, 理由同上
    Candidate fine = search(fine_pos, fine_yaws, pts_fine);
    if (fine.score < best.score) fine = best;
    result = fine;

    const double dt = (now() - t0).seconds();
    if (fine.score < accept_score_) {
      RCLCPP_WARN(get_logger(),
          "重定位匹配分过低: %.2f < %.2f (耗时%.1fs), 不发布位姿", fine.score, accept_score_, dt);
      return false;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.pose.position.x = fine.x;
    msg.pose.pose.position.y = fine.y;
    msg.pose.pose.orientation.z = std::sin(fine.yaw / 2);
    msg.pose.pose.orientation.w = std::cos(fine.yaw / 2);
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 * 0.25;
    msg.pose.covariance[35] = 0.17 * 0.17;
    pose_pub_->publish(msg);
    last_reloc_time_ = now();
    RCLCPP_INFO(get_logger(),
        "重定位成功: x=%.2f y=%.2f yaw=%.1f° score=%.2f (耗时%.1fs)%s",
        fine.x, fine.y, fine.yaw * 180 / M_PI, fine.score, dt,
        from_service ? " [服务触发]" : "");
    return true;
  }

  // 成员
  bool auto_on_startup_, watchdog_en_;
  double accept_score_, sigma_, coarse_step_, coarse_yaw_step_;
  double max_beam_range_, min_clearance_, watchdog_score_, watchdog_sigma_;
  int num_threads_, watchdog_count_;
  bool field_ready_ = false, startup_done_ = false;
  int startup_tries_ = 0, low_score_cnt_ = 0;
  rclcpp::Time last_reloc_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<float> dist_, score_, score_tight_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::mutex scan_mutex_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoRelocalize>());
  rclcpp::shutdown();
  return 0;
}
