#include "ai_ship_robot_gazebo/cmd_vel_slope_adapter.hpp"
#include "ai_ship_robot_gazebo/drive_limits.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace
{

std::shared_ptr<ai_ship_robot_gazebo::CmdVelSlopeAdapter> make_adapter_with_high_rate_limits()
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("max_linear_acceleration_mps2", 1000.0);
  options.append_parameter_override("max_angular_acceleration_radps2", 1000.0);
  return std::make_shared<ai_ship_robot_gazebo::CmdVelSlopeAdapter>(options);
}

geometry_msgs::msg::Twist wait_for_message(
  const std::shared_ptr<rclcpp::Node> & helper_node,
  const std::shared_ptr<ai_ship_robot_gazebo::CmdVelSlopeAdapter> & adapter,
  geometry_msgs::msg::Twist published)
{
  geometry_msgs::msg::Twist received;
  bool got_message = false;
  auto subscription = helper_node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_slope_limited",
    rclcpp::QoS(10),
    [&](const geometry_msgs::msg::Twist::SharedPtr message) {
      received = *message;
      got_message = true;
    });

  // publisher/subscriber接続を待ってから投入し、単体試験でも配送タイミング差で不安定化しないようにする。
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (
    std::chrono::steady_clock::now() < deadline &&
    adapter->count_subscribers("/cmd_vel_slope_limited") == 0)
  {
    rclcpp::spin_some(helper_node);
    rclcpp::spin_some(adapter);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  adapter->handle_cmd_vel_for_test(published);
  while (std::chrono::steady_clock::now() < deadline && !got_message) {
    rclcpp::spin_some(helper_node);
    rclcpp::spin_some(adapter);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(got_message);
  return received;
}

}  // namespace

TEST(CmdVelSlopeAdapter, ClearsLateralComponent)
{
  auto adapter = make_adapter_with_high_rate_limits();
  auto helper_node = std::make_shared<rclcpp::Node>("cmd_vel_slope_adapter_test_helper_lateral");

  geometry_msgs::msg::Twist input;
  input.linear.x = 0.6;
  input.linear.y = 0.3;
  input.angular.z = 0.1;

  // 坂道検証モードでは横成分だけを落とし、前進と旋回はそのまま維持する。
  const auto output = wait_for_message(helper_node, adapter, input);
  EXPECT_DOUBLE_EQ(output.linear.x, 0.6);
  EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.1);
}

TEST(CmdVelSlopeAdapter, ClampsLinearAndAngularSpeed)
{
  auto adapter = make_adapter_with_high_rate_limits();
  auto helper_node = std::make_shared<rclcpp::Node>("cmd_vel_slope_adapter_test_helper_limit");

  geometry_msgs::msg::Twist input;
  input.linear.x = 2.0;
  input.linear.y = 0.0;
  input.angular.z = 1.5;

  // controller直前でも上限制約を再適用し、外部publish経路の過大入力を防ぐ。
  const auto output = wait_for_message(helper_node, adapter, input);
  EXPECT_DOUBLE_EQ(output.linear.x, ai_ship_robot_gazebo::kMaxTranslationSpeedMps);
  EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
  EXPECT_NEAR(output.angular.z, ai_ship_robot_gazebo::kMaxYawRateRadPerSec, 1e-12);
}

TEST(CmdVelSlopeAdapter, NormalizesCompositeTranslationAfterLateralRemoval)
{
  auto adapter = make_adapter_with_high_rate_limits();
  auto helper_node = std::make_shared<rclcpp::Node>("cmd_vel_slope_adapter_test_helper_composite");

  geometry_msgs::msg::Twist input;
  input.linear.x = 1.8;
  input.linear.y = 0.4;
  input.angular.z = 0.0;

  // まず横成分を無効化し、その後の残り入力に対して最大並進速度を保証する。
  const auto output = wait_for_message(helper_node, adapter, input);
  EXPECT_DOUBLE_EQ(output.linear.x, ai_ship_robot_gazebo::kMaxTranslationSpeedMps);
  EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(CmdVelSlopeAdapter, RampsLinearVelocityStepInput)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("max_linear_acceleration_mps2", 0.5);
  options.append_parameter_override("max_angular_acceleration_radps2", 1000.0);
  auto adapter = std::make_shared<ai_ship_robot_gazebo::CmdVelSlopeAdapter>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("cmd_vel_slope_adapter_test_helper_linear_ramp");

  geometry_msgs::msg::Twist input;
  input.linear.x = 1.4;

  // 2回目以降は前回出力から滑らかに追従し、減速側も同じレート制限に入ることを確認する。
  wait_for_message(helper_node, adapter, input);
  std::this_thread::sleep_for(120ms);
  geometry_msgs::msg::Twist decel_input;
  const auto output = wait_for_message(helper_node, adapter, decel_input);
  EXPECT_GT(output.linear.x, 0.0);
  EXPECT_LT(output.linear.x, 1.4);
  EXPECT_NEAR(output.linear.x, 1.34, 0.06);
  EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(CmdVelSlopeAdapter, RampsAngularVelocityStepInput)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("max_linear_acceleration_mps2", 1000.0);
  options.append_parameter_override("max_angular_acceleration_radps2", 0.5);
  auto adapter = std::make_shared<ai_ship_robot_gazebo::CmdVelSlopeAdapter>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("cmd_vel_slope_adapter_test_helper_angular_ramp");

  geometry_msgs::msg::Twist input;
  input.angular.z = 0.8;

  // 旋回も2回目以降の変化量が制限され、急な反転を避けることを確認する。
  wait_for_message(helper_node, adapter, input);
  std::this_thread::sleep_for(120ms);
  geometry_msgs::msg::Twist decel_input;
  const auto output = wait_for_message(helper_node, adapter, decel_input);
  EXPECT_GT(output.angular.z, 0.0);
  EXPECT_LT(output.angular.z, 0.8);
  EXPECT_NEAR(output.angular.z, 0.74, 0.06);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
