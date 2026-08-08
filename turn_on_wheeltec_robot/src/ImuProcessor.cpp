#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

//extern "C" {
//#include "turn_on_wheeltec_robot/Quaternion_Solution.h"
//}

// Returns true if |val1| < val2
bool abslt(const double& val1, const double& val2)
{
  return std::abs(val1) < val2;
}

class ImuProcessor : public rclcpp::Node
{
public:
    ImuProcessor() : Node("imu_processor")
    {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 2,std::bind(&ImuProcessor::imuCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 2,std::bind(&ImuProcessor::odomCallback, this, std::placeholders::_1));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_filtered", 2);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        linear_vel_ = msg->twist.twist.linear.x;
        angular_vel_ = msg->twist.twist.angular.z;
        odom_vaild=true;

    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(!odom_vaild) return;

       //RCLCPP_INFO(this->get_logger(), "linear_vel: %f, angular_vel: %f", linear_vel_, angular_vel_);

        bool is_static = abslt(linear_vel_,x_vel_threshold) && abslt(angular_vel_,z_vel_threshold);
        
        if(is_static){
            static_count++;
            dynamic_count=0;
        }
        else{
            dynamic_count++;
            static_count=0;
        }
        imu_freeze = abslt(static_threshold,static_count);

        // ---------------- 陀螺零偏估计与补偿 (2026-08-07 新增) ----------------
        // 【为什么必须做】本车 IMU 是底盘 STM32 板载的, 出厂不做零偏标定, 每次上电
        // 的零点都不一样。2026-08-07 实测: 车完全静止(/odom 偏航 32 秒 +0.00°),
        // 陀螺 z 却稳定输出 -1.571°/s(标准差仅 0.020°/s, 是恒定偏置不是噪声),
        // EKF 的 imu0_config 里 vyaw=true 把它一路积分 -> 偏航每分钟漂 94°,
        // 表现为激光点云不停旋转、AMCL 怎么拉都拉不住、定位持续漂移。
        //
        // 【为什么放在这里】本节点已经有现成的静止判据(is_static, 用 odom 双阈值 +
        // 连续计数去抖), 之前只拿去冻结姿态解算, 没用来估零偏 —— 万事俱备。
        // 补完之后 /imu/data_filtered 才名副实, EKF 配置一行都不用改。
        //
        // 【为什么先锁定再慢跟】刚静止的前若干帧可能还有余振, 所以要连续静止
        // static_threshold 帧之后才开始采; 锁定用简单平均(收敛快), 之后用很慢的
        // 指数平滑跟随温漂(alpha 极小), 避免车"缓慢匀速转弯"时被误当成静止而把
        // 真实角速度吃进零偏 —— 那会让车永远转不到位。
        if(is_static && imu_freeze){
            const double gz = msg->angular_velocity.z;
            if(!bias_ready_){
                bias_acc_x_ += msg->angular_velocity.x;
                bias_acc_y_ += msg->angular_velocity.y;
                bias_acc_z_ += gz;
                if(++bias_n_ >= bias_lock_n_){
                    gx_bias_ = bias_acc_x_/bias_n_;
                    gy_bias_ = bias_acc_y_/bias_n_;
                    gz_bias_ = bias_acc_z_/bias_n_;
                    bias_ready_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "陀螺零偏标定完成(%d帧静止): x=%.5f y=%.5f z=%.5f rad/s "
                        "(z 折合 %.3f deg/s, 即每分钟 %.1f度); 已从 /imu/data_filtered 扣除",
                        bias_n_, gx_bias_, gy_bias_, gz_bias_,
                        gz_bias_*57.2958, gz_bias_*57.2958*60.0);
                }
            }else{
                // 慢跟温漂。alpha 很小(约几分钟时间常数), 不会被短时误判带跑
                gx_bias_ += bias_alpha_*(msg->angular_velocity.x - gx_bias_);
                gy_bias_ += bias_alpha_*(msg->angular_velocity.y - gy_bias_);
                gz_bias_ += bias_alpha_*(gz - gz_bias_);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                    "陀螺零偏(慢跟): z=%.5f rad/s (%.3f deg/s)",
                    gz_bias_, gz_bias_*57.2958);
            }
        }
        // 未标定完成前不补偿(宁可不改也不要用一个错的零偏)
        const double wx = msg->angular_velocity.x - (bias_ready_ ? gx_bias_ : 0.0);
        const double wy = msg->angular_velocity.y - (bias_ready_ ? gy_bias_ : 0.0);
        const double wz = msg->angular_velocity.z - (bias_ready_ ? gz_bias_ : 0.0);

        //imu_freeze = is_static ? 1 : 0;
        //姿态解算
        Quaternion_Solution(
            wx, wy, wz,
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        sensor_msgs::msg::Imu out = *msg;

        // **这三行是关键**: 原来 out 是整条复制、角速度原样透传, 所以
        // /imu/data_filtered 和 /imu/data_raw 逐帧完全相同(实测均值标准差全同),
        // "filtered" 名不副实, EKF 吃到的还是带零偏的原始值。
        out.angular_velocity.x = wx;
        out.angular_velocity.y = wy;
        out.angular_velocity.z = wz;

        //使用Quaternion_Solution中的全局姿态
        out.orientation.w = q0;
        out.orientation.x = q1;
        out.orientation.y = q2;
        out.orientation.z = q3;

        imu_pub_->publish(out);
    }

    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;

    // 陀螺零偏(rad/s)与标定状态
    double gx_bias_=0.0, gy_bias_=0.0, gz_bias_=0.0;
    double bias_acc_x_=0.0, bias_acc_y_=0.0, bias_acc_z_=0.0;
    int    bias_n_=0;
    bool   bias_ready_=false;
    const int    bias_lock_n_=200;    // 连续静止满这么多帧才锁定初值
    const double bias_alpha_=2e-5;    // 锁定后的慢跟系数(时间常数约几分钟)

    double x_vel_threshold=0.05;
    double z_vel_threshold=0.05;

    bool odom_vaild=false;
    int static_count=0;
    int dynamic_count=0;
    int static_threshold=10;
    int dynamic_threshold=3;


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<ImuProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;  
} 
