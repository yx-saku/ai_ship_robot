#pragma once

#include <string>
#include <vector>

namespace ai_ship_robot_gazebo
{

enum class ScenarioStepType
{
  Velocity,
  MoveToPose,
};

enum class MoveToPoseMode
{
  Absolute,
  Relative,
};

struct MoveTarget
{
  double x{};
  double y{};
  double tolerance{};
};

struct PoseTarget
{
  double yaw{};
  double tolerance{};
};

struct MoveToPoseTarget
{
  MoveToPoseMode mode{MoveToPoseMode::Absolute};
  bool has_position{};
  bool has_yaw{};
  MoveTarget position{};
  PoseTarget yaw{};
};

struct ScenarioStep
{
  ScenarioStepType type{ScenarioStepType::Velocity};
  double duration_sec{};
  double linear_x{};
  double linear_y{};
  double angular_z{};
  MoveToPoseTarget move_to_pose{};
};

struct ScenarioDefinition
{
  std::vector<ScenarioStep> steps;
};

ScenarioDefinition load_scenario_file(const std::string & scenario_file);

std::string format_step_command(const ScenarioStep & step);

}  // namespace ai_ship_robot_gazebo
