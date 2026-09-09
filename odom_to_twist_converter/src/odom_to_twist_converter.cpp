#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class OdomToTwistConverter : public rclcpp::Node
{
public:
  OdomToTwistConverter()
  : Node("odom_to_twist_converter")
  {
    // 레거시 km/h bag 재생 보정용. bridge 가 이미 m/s 로 발행하면 1.0(기본).
    vel_scale_ = this->declare_parameter("vel_scale", 1.0);

    // Optional synthetic sensor noise on the shared twist (odometer vx, gyro wz).
    // The MORAI ground-truth odom carries no measurement noise (Table 2:
    // odometer/IMU noise unset), so the three sub-EKFs share a *noiseless*
    // twist and therefore have no common-mode error. Injecting a realistic
    // odometer/gyro noise here — from a SINGLE converter feeding all three
    // gyro_odometer instances — creates a genuine shared measurement error,
    // which is what the HG-SCIF split covariance is designed to handle.
    // Default 0.0 keeps the deployed (noiseless) behaviour bit-for-bit.
    noise_vx_std_ = this->declare_parameter("noise_vx_std", 0.0);   // [m/s]
    noise_wz_std_ = this->declare_parameter("noise_wz_std", 0.0);   // [rad/s]
    const int seed = this->declare_parameter("noise_seed", 0);
    rng_.seed(static_cast<unsigned>(seed));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/morai/ground_truth/odom", 1,
      std::bind(&OdomToTwistConverter::odomCallback, this, std::placeholders::_1));

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/vehicle/twist_with_covariance", 1);

    RCLCPP_INFO(
      this->get_logger(),
      "Odom -> TwistWithCovariance converter started (noise vx=%.3f wz=%.3f seed=%d).",
      noise_vx_std_, noise_wz_std_, seed);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out_msg;
    out_msg.header = msg->header;
    out_msg.twist = msg->twist;  // Odometry -> use twist
    out_msg.twist.twist.linear.x *= vel_scale_;
    out_msg.twist.twist.linear.y *= vel_scale_;
    out_msg.twist.twist.linear.z *= vel_scale_;

    if (noise_vx_std_ > 0.0) {
      out_msg.twist.twist.linear.x +=
        std::normal_distribution<double>(0.0, noise_vx_std_)(rng_);
    }
    if (noise_wz_std_ > 0.0) {
      out_msg.twist.twist.angular.z +=
        std::normal_distribution<double>(0.0, noise_wz_std_)(rng_);
    }

    pub_twist_->publish(out_msg);
  }

  double vel_scale_{1.0};
  double noise_vx_std_{0.0};
  double noise_wz_std_{0.0};
  std::mt19937 rng_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTwistConverter>());
  rclcpp::shutdown();
  return 0;
}
