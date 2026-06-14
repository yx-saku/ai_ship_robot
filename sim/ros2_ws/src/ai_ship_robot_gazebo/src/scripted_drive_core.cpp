#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ai_ship_robot_gazebo
{
namespace
{

struct CommandAxis
{
  const char * axis_name;
  double ScenarioStep::* member;
  double direction;
};

const std::unordered_map<std::string, CommandAxis> & command_axes()
{
  static const std::unordered_map<std::string, CommandAxis> axes{
    {"forward", {"linear_x", &ScenarioStep::linear_x, 1.0}},
    {"backward", {"linear_x", &ScenarioStep::linear_x, -1.0}},
    {"left", {"linear_y", &ScenarioStep::linear_y, 1.0}},
    {"right", {"linear_y", &ScenarioStep::linear_y, -1.0}},
    {"yaw_left", {"angular_z", &ScenarioStep::angular_z, 1.0}},
    {"yaw_right", {"angular_z", &ScenarioStep::angular_z, -1.0}},
  };
  return axes;
}

template<typename T>
T required_as(const YAML::Node & node, const std::string & key, const std::string & message)
{
  if (!node[key]) {
    throw std::invalid_argument(message);
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(message);
  }
}

void apply_command(
  ScenarioStep & step, std::unordered_map<std::string, std::string> & used_axes,
  const YAML::Node & raw_command, const std::size_t step_index, const std::size_t command_index,
  bool & stop_used)
{
  if (!raw_command.IsMap()) {
    throw std::invalid_argument(
      "steps[" + std::to_string(step_index) + "].commands[" + std::to_string(command_index) +
      "] must be a mapping.");
  }

  const auto command_type = required_as<std::string>(
    raw_command, "type",
    "steps[" + std::to_string(step_index) + "].commands[" + std::to_string(command_index) +
    "] is missing type.");
  if (command_type == "stop") {
    stop_used = true;
    return;
  }

  const auto axis_iter = command_axes().find(command_type);
  if (axis_iter == command_axes().end()) {
    throw std::invalid_argument("Unsupported command type at steps[" + std::to_string(step_index) + "]: " + command_type);
  }

  // 同一軸の競合は曖昧なTwistを作るため、読み込み時に即時拒否する。
  const auto speed = required_as<double>(
    raw_command, "speed",
    "steps[" + std::to_string(step_index) + "].commands[" + std::to_string(command_index) +
    "].speed must be numeric.");
  if (speed < 0.0) {
    throw std::invalid_argument(
      "steps[" + std::to_string(step_index) + "].commands[" + std::to_string(command_index) +
      "].speed must be >= 0.");
  }

  const auto & axis = axis_iter->second;
  if (used_axes.find(axis.axis_name) != used_axes.end()) {
    throw std::invalid_argument(
      "steps[" + std::to_string(step_index) + "] cannot combine " + used_axes[axis.axis_name] +
      " and " + command_type + " on " + axis.axis_name + ".");
  }
  used_axes.emplace(axis.axis_name, command_type);
  step.*(axis.member) = axis.direction * speed;
}

ScenarioStep parse_step(const YAML::Node & raw_step, const std::size_t step_index)
{
  if (!raw_step.IsMap()) {
    throw std::invalid_argument("steps[" + std::to_string(step_index) + "] must be a mapping.");
  }

  ScenarioStep step{};
  step.duration_sec = required_as<double>(
    raw_step, "duration_sec", "steps[" + std::to_string(step_index) + "].duration_sec must be numeric.");
  if (step.duration_sec <= 0.0) {
    throw std::invalid_argument("steps[" + std::to_string(step_index) + "].duration_sec must be > 0.");
  }

  const auto commands = raw_step["commands"];
  if (!commands || !commands.IsSequence() || commands.size() == 0) {
    throw std::invalid_argument("steps[" + std::to_string(step_index) + "].commands must be a non-empty list.");
  }

  // 各ステップはゼロ初期化から合成し、前ステップの速度持ち越しを防ぐ。
  bool stop_used = false;
  std::unordered_map<std::string, std::string> used_axes;
  for (std::size_t index = 0; index < commands.size(); ++index) {
    apply_command(step, used_axes, commands[index], step_index, index + 1, stop_used);
  }
  if (stop_used && commands.size() != 1) {
    throw std::invalid_argument("steps[" + std::to_string(step_index) + "] stop must be the only command in the step.");
  }

  return step;
}

}  // namespace

std::vector<ScenarioStep> load_scenario_file(const std::string & scenario_file)
{
  YAML::Node data;
  try {
    data = YAML::LoadFile(scenario_file);
  } catch (const YAML::Exception & exc) {
    throw std::invalid_argument("Scenario file not found or invalid: " + scenario_file + ": " + exc.what());
  }

  const auto steps_node = data["steps"];
  if (!data.IsMap() || !steps_node || !steps_node.IsSequence()) {
    throw std::invalid_argument("Scenario YAML must contain a top-level 'steps' list.");
  }

  // 起動直後に全ステップを検証し、走行中の設定エラーで停止しないようにする。
  std::vector<ScenarioStep> steps;
  steps.reserve(steps_node.size());
  for (std::size_t index = 0; index < steps_node.size(); ++index) {
    steps.push_back(parse_step(steps_node[index], index + 1));
  }
  if (steps.empty()) {
    throw std::invalid_argument("Scenario YAML must define at least one step.");
  }

  return steps;
}

std::string format_step_command(const ScenarioStep & step)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "linear_x=" << step.linear_x << ", linear_y="
         << step.linear_y << ", angular_z=" << step.angular_z << std::setprecision(2)
         << ", duration=" << step.duration_sec << "s";
  return stream.str();
}

}  // namespace ai_ship_robot_gazebo
