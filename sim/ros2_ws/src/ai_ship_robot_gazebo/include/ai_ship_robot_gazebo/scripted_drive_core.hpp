#pragma once

#include <string>
#include <vector>

namespace ai_ship_robot_gazebo
{

struct ScenarioStep
{
  double duration_sec{};
  double linear_x{};
  double linear_y{};
  double angular_z{};
};

struct ScenarioDefinition
{
  std::vector<ScenarioStep> steps;
};

ScenarioDefinition load_scenario_file(const std::string & scenario_file);

std::string format_step_command(const ScenarioStep & step);

}  // namespace ai_ship_robot_gazebo
