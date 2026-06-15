#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace ai_ship_robot_slam
{
namespace
{
using Imu = sensor_msgs::msg::Imu;

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double vector_norm(const geometry_msgs::msg::Vector3 & vector)
{
  return std::sqrt(
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

bool finite_vector(const geometry_msgs::msg::Vector3 & vector)
{
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}
}  // namespace

class SixAxisImuInitialOrientationNode : public rclcpp::Node
{
public:
  SixAxisImuInitialOrientationNode()
  : Node("six_axis_imu_initial_orientation_node")
  {
    // 入出力topicと静止判定条件をparameter化し、実機ログに合わせて閾値だけ調整できるようにする。
    input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/livox/imu");
    output_imu_topic_ = declare_parameter<std::string>("output_imu_topic", "/livox/imu_oriented");
    expected_acceleration_norm_ = declare_parameter<double>("expected_acceleration_norm", 1.0);
    acceleration_norm_tolerance_ = declare_parameter<double>("acceleration_norm_tolerance", 0.35);
    max_initial_angular_velocity_rad_s_ = declare_parameter<double>(
      "max_initial_angular_velocity_rad_s", 0.2);
    min_initial_samples_ = declare_parameter<int>("min_initial_samples", 50);
    min_initial_duration_sec_ = declare_parameter<double>("min_initial_duration_sec", 0.5);
    reset_on_motion_ = declare_parameter<bool>("reset_on_motion", true);
    publish_before_initialized_ = declare_parameter<bool>("publish_before_initialized", false);
    orientation_roll_pitch_variance_ = declare_parameter<double>("orientation_roll_pitch_variance", 0.01);
    orientation_yaw_variance_ = declare_parameter<double>("orientation_yaw_variance", 1.0e6);

    if (expected_acceleration_norm_ <= 0.0) {
      throw std::runtime_error("expected_acceleration_norm must be positive.");
    }
    if (min_initial_samples_ <= 0) {
      throw std::runtime_error("min_initial_samples must be positive.");
    }
    if (min_initial_duration_sec_ < 0.0) {
      throw std::runtime_error("min_initial_duration_sec must be non-negative.");
    }

    const auto qos = rclcpp::SensorDataQoS();
    imu_pub_ = create_publisher<Imu>(output_imu_topic_, qos);
    imu_sub_ = create_subscription<Imu>(
      input_imu_topic_, qos,
      std::bind(&SixAxisImuInitialOrientationNode::handle_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Estimate initial 6-axis IMU orientation: input=%s output=%s",
      input_imu_topic_.c_str(), output_imu_topic_.c_str());
  }

private:
  void handle_imu(const Imu::SharedPtr input)
  {
    if (!initialized_) {
      update_initial_estimate(*input);
      if (!initialized_ && !publish_before_initialized_) {
        return;
      }
    }

    publish_with_orientation(*input);
  }

  void update_initial_estimate(const Imu & input)
  {
    if (!sample_is_stationary(input)) {
      if (reset_on_motion_) {
        reset_accumulator();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for stationary IMU samples to estimate initial roll/pitch.");
      return;
    }

    // 静止条件を満たす連続サンプルだけを平均し、並進加速度が混ざった初期化を避ける。
    const double stamp_sec = stamp_to_seconds(input.header.stamp);
    if (sample_count_ == 0 || stamp_sec < first_sample_time_sec_) {
      first_sample_time_sec_ = stamp_sec;
      sum_acc_x_ = 0.0;
      sum_acc_y_ = 0.0;
      sum_acc_z_ = 0.0;
      sample_count_ = 0;
    }

    sum_acc_x_ += input.linear_acceleration.x;
    sum_acc_y_ += input.linear_acceleration.y;
    sum_acc_z_ += input.linear_acceleration.z;
    sample_count_ += 1;

    const double duration_sec = stamp_sec - first_sample_time_sec_;
    if (sample_count_ >= min_initial_samples_ && duration_sec >= min_initial_duration_sec_) {
      initialize_orientation();
    }
  }

  bool sample_is_stationary(const Imu & input) const
  {
    if (!finite_vector(input.linear_acceleration) || !finite_vector(input.angular_velocity)) {
      return false;
    }

    // 加速度ノルムと角速度の小ささを併用し、走行中の並進加速度を重力と誤認しないようにする。
    const double acceleration_norm = vector_norm(input.linear_acceleration);
    const double angular_velocity_norm = vector_norm(input.angular_velocity);
    const double acceleration_error = std::abs(acceleration_norm - expected_acceleration_norm_);
    return acceleration_error <= acceleration_norm_tolerance_ &&
           angular_velocity_norm <= max_initial_angular_velocity_rad_s_;
  }

  void initialize_orientation()
  {
    const double count = static_cast<double>(sample_count_);
    const double acc_x = sum_acc_x_ / count;
    const double acc_y = sum_acc_y_ / count;
    const double acc_z = sum_acc_z_ / count;
    const double horizontal_norm = std::hypot(acc_y, acc_z);

    if (!std::isfinite(horizontal_norm) || horizontal_norm <= std::numeric_limits<double>::epsilon()) {
      reset_accumulator();
      RCLCPP_WARN(get_logger(), "Initial acceleration average is degenerate; retry orientation estimate.");
      return;
    }

    // 6軸IMUではyawは観測できないため0に固定し、重力方向からroll/pitchだけを決める。
    initial_roll_ = std::atan2(acc_y, acc_z);
    initial_pitch_ = std::atan2(-acc_x, horizontal_norm);
    initialized_ = true;

    RCLCPP_INFO(
      get_logger(), "Estimated initial IMU orientation: roll=%.6f pitch=%.6f yaw=0.000000 from %d samples",
      initial_roll_, initial_pitch_, sample_count_);
  }

  void publish_with_orientation(const Imu & input)
  {
    Imu output = input;
    tf2::Quaternion orientation;

    if (initialized_) {
      orientation.setRPY(initial_roll_, initial_pitch_, 0.0);
    } else {
      orientation.setRPY(0.0, 0.0, 0.0);
    }
    orientation.normalize();

    output.orientation.x = orientation.x();
    output.orientation.y = orientation.y();
    output.orientation.z = orientation.z();
    output.orientation.w = orientation.w();

    // orientationは初期roll/pitch用でyawは未知なので、yaw分散を大きくして下流へ意図を残す。
    output.orientation_covariance = {
      orientation_roll_pitch_variance_, 0.0, 0.0,
      0.0, orientation_roll_pitch_variance_, 0.0,
      0.0, 0.0, orientation_yaw_variance_};
    imu_pub_->publish(output);
  }

  void reset_accumulator()
  {
    sum_acc_x_ = 0.0;
    sum_acc_y_ = 0.0;
    sum_acc_z_ = 0.0;
    sample_count_ = 0;
    first_sample_time_sec_ = 0.0;
  }

  std::string input_imu_topic_;
  std::string output_imu_topic_;
  double expected_acceleration_norm_;
  double acceleration_norm_tolerance_;
  double max_initial_angular_velocity_rad_s_;
  int min_initial_samples_;
  double min_initial_duration_sec_;
  bool reset_on_motion_;
  bool publish_before_initialized_;
  double orientation_roll_pitch_variance_;
  double orientation_yaw_variance_;

  bool initialized_{false};
  double initial_roll_{0.0};
  double initial_pitch_{0.0};
  double sum_acc_x_{0.0};
  double sum_acc_y_{0.0};
  double sum_acc_z_{0.0};
  int sample_count_{0};
  double first_sample_time_sec_{0.0};

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::SixAxisImuInitialOrientationNode>());
  rclcpp::shutdown();
  return 0;
}
