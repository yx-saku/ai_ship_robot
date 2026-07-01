#pragma once

#include <algorithm>
#include <cmath>

namespace ai_ship_robot_gazebo
{

struct DriveCommand
{
  double linear_x{};
  double linear_y{};
  double angular_z{};
};

struct DriveLimitResult
{
  DriveCommand command;
  bool linear_limited{};
  bool angular_limited{};
};

constexpr double kMaxTranslationSpeedMps = 1.4;
constexpr double kMaxYawRateRadPerSec = 0.873;  // 50 deg/s

inline DriveLimitResult apply_drive_limits(
  const double linear_x,
  const double linear_y,
  const double angular_z)
{
  DriveLimitResult result{};
  result.command.linear_x = linear_x;
  result.command.linear_y = linear_y;
  result.command.angular_z = std::clamp(
    angular_z,
    -kMaxYawRateRadPerSec,
    kMaxYawRateRadPerSec);
  result.angular_limited = result.command.angular_z != angular_z;

  const double translation_norm = std::hypot(linear_x, linear_y);
  if (translation_norm > kMaxTranslationSpeedMps && translation_norm > 0.0) {
    const double scale = kMaxTranslationSpeedMps / translation_norm;
    result.command.linear_x = linear_x * scale;
    result.command.linear_y = linear_y * scale;
    result.linear_limited = true;
  }

  return result;
}

}  // namespace ai_ship_robot_gazebo
