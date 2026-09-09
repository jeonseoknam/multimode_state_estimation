#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include <atomic>
#include <cmath>
#include <string>

class PoseConverter : public rclcpp::Node
{
public:
  PoseConverter() : Node("pose_to_pose_with_cov")
  {
    // -------------------------------
    // Parameters
    // -------------------------------
    declare_parameter<std::string>("input_topic", "/current_pose");
    declare_parameter<std::string>("output_topic", "/ndt_pose_with_covariance");

    declare_parameter<std::string>("expected_frame_id", "map");

    declare_parameter<double>("sigma_xy_m", 0.05);      // x,y stddev [m] (isotropic fallback)
    declare_parameter<double>("sigma_yaw_deg", 1.0);    // yaw stddev [deg]

    // Anisotropic measurement noise in the VEHICLE frame. When both are
    // > 0 they override sigma_xy_m: the measurement covariance becomes
    // R_vehicle = diag(sigma_lon^2, sigma_lat^2), rotated into the map
    // frame by the measurement yaw. This lets the along-track (longitudinal)
    // uncertainty be set larger than the cross-track (lateral) one, matching
    // the real error structure (e.g. NDT slip ≫ lateral) so the EKF output
    // covariance is consistent without globally distrusting the measurement.
    // Default -1 keeps the original isotropic sigma_xy_m behaviour.
    declare_parameter<double>("sigma_lon_m", -1.0);     // longitudinal (along-track) stddev [m]
    declare_parameter<double>("sigma_lat_m", -1.0);     // lateral (cross-track) stddev [m]

    declare_parameter<bool>("use_zrp", false);          // z/roll/pitch 관측 포함 여부
    declare_parameter<double>("sigma_z_m", 0.05);       // z stddev [m]
    declare_parameter<double>("sigma_rp_deg", 1.0);     // roll/pitch stddev [deg]

    declare_parameter<double>("big_unc", 1e6);          // 관측하지 않는 축의 큰 분산

    // Optional health gate: when set, this node only forwards measurements
    // while the boolean published on `health_topic` is True. Used to prevent
    // a faulty raw pose (during a GUI fault injection) from contaminating the
    // EKF state — without this, the EKF eventually accepts the bad
    // measurement and then takes a long time to catch up after recovery.
    // Empty string disables the gate (preserves original behaviour).
    declare_parameter<std::string>("health_topic", "");

    // Optional input decimation (throttle): publish only every Nth pose.
    // For temporally-correlated sources (e.g. ORB-SLAM3 ~32Hz, decorr ~1-4s),
    // the downstream EKF over-averages correlated measurements and its output
    // covariance collapses (overconfident). Decimating to ~5Hz (decimation~6)
    // breaks the correlation so the EKF stays consistent WITHOUT output scaling
    // (verified offline 2026-06-25: 5Hz + sigma 0.10 → 2σ inside ~89%). 1 = off.
    declare_parameter<int>("decimation", 1);

    const std::string input_topic = get_parameter("input_topic").as_string();
    const std::string output_topic = get_parameter("output_topic").as_string();
    const std::string expected_frame = get_parameter("expected_frame_id").as_string();
    const std::string health_topic = get_parameter("health_topic").as_string();

    // -------------------------------
    // Subscriber
    // SensorDataQoS로 맞춰서
    // /current_pose, /gnss_pose_localcartesian 둘 다 안전하게 받기
    // -------------------------------
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic,
      rclcpp::SensorDataQoS().keep_last(64),
      std::bind(&PoseConverter::callback, this, std::placeholders::_1));

    // -------------------------------
    // Publisher
    // -------------------------------
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic,
      rclcpp::QoS(10));

    // -------------------------------
    // Optional health gate subscriber
    // -------------------------------
    if (!health_topic.empty()) {
      sub_health_ = create_subscription<std_msgs::msg::Bool>(
        health_topic,
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          this->healthy_.store(msg->data);
        });
    }

    RCLCPP_INFO(this->get_logger(), "pose_to_pose_with_cov started");
    RCLCPP_INFO(this->get_logger(), "  input_topic       : %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  output_topic      : %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  expected_frame_id : %s", expected_frame.c_str());
    if (!health_topic.empty()) {
      RCLCPP_INFO(this->get_logger(), "  health_topic      : %s (gate enabled)",
        health_topic.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "  health_topic      : (none — gate disabled)");
    }
  }

private:
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Health gate: if a health_topic was configured and it is currently
    // False, suppress this measurement entirely so the downstream EKF state
    // is not corrupted by the faulty pose. EKF will simply run
    // prediction-only during the fault, then resume normally as soon as the
    // first post-recovery measurement comes through (state hasn't been
    // dragged toward the fault, so the Mahalanobis gate passes immediately).
    if (!healthy_.load()) {
      return;
    }

    // Input decimation (throttle): drop all but every Nth healthy pose.
    const int decimation = static_cast<int>(std::max<int64_t>(1, get_parameter("decimation").as_int()));
    if ((decim_count_++ % static_cast<unsigned long>(decimation)) != 0) {
      return;
    }

    const std::string expected_frame = get_parameter("expected_frame_id").as_string();

    const double sigma_xy_m    = get_parameter("sigma_xy_m").as_double();
    const double sigma_yaw_deg = get_parameter("sigma_yaw_deg").as_double();
    const double sigma_lon_m   = get_parameter("sigma_lon_m").as_double();
    const double sigma_lat_m   = get_parameter("sigma_lat_m").as_double();

    const bool use_zrp         = get_parameter("use_zrp").as_bool();
    const double sigma_z_m     = get_parameter("sigma_z_m").as_double();
    const double sigma_rp_deg  = get_parameter("sigma_rp_deg").as_double();

    const double big_unc       = get_parameter("big_unc").as_double();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.header.frame_id = expected_frame;
    out.pose.pose = msg->pose;

    // covariance 초기화
    for (double & v : out.pose.covariance) {
      v = 0.0;
    }

    // index:
    // 0:x, 1:xy, 6:yx, 7:y, 14:z, 21:roll, 28:pitch, 35:yaw
    const double var_yaw = std::pow(sigma_yaw_deg * M_PI / 180.0, 2.0);

    if (sigma_lon_m > 0.0 && sigma_lat_m > 0.0) {
      // Anisotropic R in the vehicle frame, rotated into the map frame by
      // the measurement yaw:
      //   R_map = Rz(yaw) * diag(var_lon, var_lat) * Rz(yaw)^T
      const auto & q = msg->pose.orientation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double yaw  = std::atan2(siny, cosy);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      const double var_lon = sigma_lon_m * sigma_lon_m;
      const double var_lat = sigma_lat_m * sigma_lat_m;
      const double xx = c * c * var_lon + s * s * var_lat;
      const double yy = s * s * var_lon + c * c * var_lat;
      const double xy = c * s * (var_lon - var_lat);

      out.pose.covariance[0] = xx;   // x
      out.pose.covariance[1] = xy;   // xy
      out.pose.covariance[6] = xy;   // yx
      out.pose.covariance[7] = yy;   // y
    } else {
      // Isotropic fallback (original behaviour).
      const double var_xy = sigma_xy_m * sigma_xy_m;
      out.pose.covariance[0] = var_xy;   // x
      out.pose.covariance[7] = var_xy;   // y
    }
    out.pose.covariance[35] = var_yaw;  // yaw

    if (use_zrp) {
      const double var_z  = sigma_z_m * sigma_z_m;
      const double var_rp = std::pow(sigma_rp_deg * M_PI / 180.0, 2.0);

      out.pose.covariance[14] = var_z;   // z
      out.pose.covariance[21] = var_rp;  // roll
      out.pose.covariance[28] = var_rp;  // pitch
    } else {
      // 관측하지 않는 축은 크게 둬서 사실상 무시
      out.pose.covariance[14] = big_unc; // z
      out.pose.covariance[21] = big_unc; // roll
      out.pose.covariance[28] = big_unc; // pitch
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_health_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

  // True = pass measurements through. Defaults to True so that when the
  // gate is disabled (no health_topic) or the health publisher hasn't sent
  // its first message yet, the node still forwards data.
  std::atomic<bool> healthy_{true};
  unsigned long decim_count_{0};   // input decimation (throttle) counter
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConverter>());
  rclcpp::shutdown();
  return 0;
}