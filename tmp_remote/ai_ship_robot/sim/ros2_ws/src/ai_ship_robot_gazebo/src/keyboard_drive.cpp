#include "ai_ship_robot_gazebo/drive_limits.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <memory>
#include <optional>
#include <string>

namespace
{

constexpr const char * kHelpText = R"(Keyboard drive controls, TurtleBot3/teleop style:
  w or i: toggle forward component
  s or ,: toggle backward component
  j/l: toggle strafe left/right component
  a/d: toggle yaw left/right component
  Combine keys sequentially, e.g. w then a for a forward-left arc.
  Press the same component key again to clear only that component.
  space/x/k: stop all components
  Q or Esc: quit
)";

double toggle_component(const double current_value, const double next_value)
{
  return current_value == next_value ? 0.0 : next_value;
}

std::optional<char> read_key(const double timeout_sec)
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(STDIN_FILENO, &read_fds);

  timeval timeout{};
  timeout.tv_sec = static_cast<time_t>(timeout_sec);
  timeout.tv_usec = static_cast<suseconds_t>((timeout_sec - static_cast<double>(timeout.tv_sec)) * 1'000'000.0);

  const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
    return std::nullopt;
  }

  char key{};
  if (::read(STDIN_FILENO, &key, 1) != 1) {
    return std::nullopt;
  }
  return key;
}

class TerminalModeGuard
{
public:
  TerminalModeGuard()
  {
    active_ = ::tcgetattr(STDIN_FILENO, &old_settings_) == 0;
    if (active_) {
      termios new_settings = old_settings_;
      new_settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      new_settings.c_cc[VMIN] = 0;
      new_settings.c_cc[VTIME] = 0;
      ::tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    }
  }

  ~TerminalModeGuard()
  {
    if (active_) {
      ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_settings_);
    }
  }

private:
  termios old_settings_{};
  bool active_{};
};

}  // namespace

namespace ai_ship_robot_gazebo
{

class KeyboardDrive : public rclcpp::Node
{
public:
  KeyboardDrive()
  : Node("keyboard_drive")
  {
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<double>("linear_speed", kMaxTranslationSpeedMps);
    declare_parameter<double>("lateral_speed", kMaxTranslationSpeedMps);
    declare_parameter<double>("angular_speed", kMaxYawRateRadPerSec);
    declare_parameter<double>("publish_rate", 10.0);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    linear_speed_ = get_parameter("linear_speed").as_double();
    lateral_speed_ = get_parameter("lateral_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
  }

  bool handle_key(const char key)
  {
    // 同一成分キーはtoggle動作にし、円弧走行などの複合入力を維持する。
    if (key == 'w' || key == 'i') {
      linear_x_ = toggle_component(linear_x_, linear_speed_);
    } else if (key == 's' || key == ',') {
      linear_x_ = toggle_component(linear_x_, -linear_speed_);
    } else if (key == 'j') {
      linear_y_ = toggle_component(linear_y_, lateral_speed_);
    } else if (key == 'l') {
      linear_y_ = toggle_component(linear_y_, -lateral_speed_);
    } else if (key == 'a') {
      angular_z_ = toggle_component(angular_z_, angular_speed_);
    } else if (key == 'd') {
      angular_z_ = toggle_component(angular_z_, -angular_speed_);
    } else if (key == ' ' || key == 'x' || key == 'k') {
      stop();
    } else if (key == 'Q' || key == '\x1b') {
      return false;
    }
    return true;
  }

  void stop()
  {
    linear_x_ = 0.0;
    linear_y_ = 0.0;
    angular_z_ = 0.0;
    publish();
  }

  void publish()
  {
    // 複合入力時も単独入力時の最大値は維持しつつ、publish直前に合成速度だけ正規化する。
    const auto limited = apply_drive_limits(linear_x_, linear_y_, angular_z_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = limited.command.linear_x;
    twist.linear.y = limited.command.linear_y;
    twist.angular.z = limited.command.angular_z;
    publisher_->publish(twist);
  }

  double publish_rate() const
  {
    return publish_rate_;
  }

private:
  std::string cmd_vel_topic_;
  double linear_speed_{};
  double lateral_speed_{};
  double angular_speed_{};
  double publish_rate_{};
  double linear_x_{};
  double linear_y_{};
  double angular_z_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

}  // namespace ai_ship_robot_gazebo

int main(int argc, char ** argv)
{
  if (!::isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "keyboard_drive requires an interactive terminal.\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ai_ship_robot_gazebo::KeyboardDrive>();
  TerminalModeGuard terminal_guard;
  std::puts(kHelpText);

  bool keep_running = true;
  while (rclcpp::ok() && keep_running) {
    const auto key = read_key(1.0 / node->publish_rate());
    if (key.has_value()) {
      keep_running = node->handle_key(*key);
    }
    node->publish();
    rclcpp::spin_some(node);
  }

  node->stop();
  rclcpp::shutdown();
  return 0;
}
