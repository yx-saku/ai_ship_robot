#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
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

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultPositionTolerance = 0.05;
constexpr double kDefaultYawToleranceDegrees = 1.0;

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

double degrees_to_radians(const double degrees)
{
  return degrees * kPi / 180.0;
}

double radians_to_degrees(const double radians)
{
  return radians * 180.0 / kPi;
}

bool is_identifier_start(const char character)
{
  return std::isalpha(static_cast<unsigned char>(character)) != 0 || character == '_';
}

bool is_identifier_character(const char character)
{
  return std::isalnum(static_cast<unsigned char>(character)) != 0 || character == '_';
}

bool is_expansion_syntax(const std::string & text)
{
  return text.size() >= 4 && text.rfind("${", 0) == 0 && text.back() == '}';
}

std::string unwrap_expansion_syntax(const std::string & text, const std::string & context)
{
  if (!is_expansion_syntax(text)) {
    throw std::invalid_argument(context + " must use ${...} for expression expansion.");
  }
  return text.substr(2, text.size() - 3);
}

class ExpressionParser
{
public:
  ExpressionParser(
    const std::string & expression, const std::unordered_map<std::string, double> & variables,
    const std::string & context)
  : expression_(expression), variables_(variables), context_(context)
  {
  }

  double parse()
  {
    const auto value = parse_expression();
    skip_spaces();
    if (position_ != expression_.size()) {
      throw std::invalid_argument(context_ + " has unexpected token near: " + expression_.substr(position_));
    }
    return value;
  }

private:
  double parse_expression()
  {
    auto value = parse_term();
    while (true) {
      skip_spaces();
      if (match('+')) {
        value += parse_term();
        continue;
      }
      if (match('-')) {
        value -= parse_term();
        continue;
      }
      return value;
    }
  }

  double parse_term()
  {
    auto value = parse_factor();
    while (true) {
      skip_spaces();
      if (match('*')) {
        value *= parse_factor();
        continue;
      }
      if (match('/')) {
        const auto divisor = parse_factor();
        if (std::abs(divisor) <= std::numeric_limits<double>::epsilon()) {
          throw std::invalid_argument(context_ + " must not divide by zero.");
        }
        value /= divisor;
        continue;
      }
      return value;
    }
  }

  double parse_factor()
  {
    skip_spaces();
    if (match('+')) {
      return parse_factor();
    }
    if (match('-')) {
      return -parse_factor();
    }
    if (match('(')) {
      const auto value = parse_expression();
      skip_spaces();
      if (!match(')')) {
        throw std::invalid_argument(context_ + " is missing closing ')' .");
      }
      return value;
    }
    if (position_ >= expression_.size()) {
      throw std::invalid_argument(context_ + " is missing a value.");
    }
    if (is_identifier_start(expression_[position_])) {
      return parse_identifier();
    }
    return parse_number();
  }

  double parse_identifier()
  {
    const auto start = position_;
    ++position_;
    while (position_ < expression_.size() && is_identifier_character(expression_[position_])) {
      ++position_;
    }
    const auto name = expression_.substr(start, position_ - start);
    const auto iter = variables_.find(name);
    if (iter == variables_.end()) {
      throw std::invalid_argument(context_ + " references undefined variable: " + name);
    }
    return iter->second;
  }

  double parse_number()
  {
    const auto start = position_;
    bool dot_seen = false;
    while (position_ < expression_.size()) {
      const auto character = expression_[position_];
      if (std::isdigit(static_cast<unsigned char>(character)) != 0) {
        ++position_;
        continue;
      }
      if (character == '.' && !dot_seen) {
        dot_seen = true;
        ++position_;
        continue;
      }
      if ((character == 'e' || character == 'E') && position_ + 1 < expression_.size()) {
        ++position_;
        if (expression_[position_] == '+' || expression_[position_] == '-') {
          ++position_;
        }
        while (position_ < expression_.size() && std::isdigit(static_cast<unsigned char>(expression_[position_])) != 0) {
          ++position_;
        }
      }
      break;
    }
    if (start == position_) {
      throw std::invalid_argument(context_ + " contains an invalid token near: " + expression_.substr(position_));
    }
    try {
      return std::stod(expression_.substr(start, position_ - start));
    } catch (const std::exception &) {
      throw std::invalid_argument(context_ + " must be numeric or an arithmetic expression.");
    }
  }

  void skip_spaces()
  {
    while (position_ < expression_.size() && std::isspace(static_cast<unsigned char>(expression_[position_])) != 0) {
      ++position_;
    }
  }

  bool match(const char expected)
  {
    if (position_ < expression_.size() && expression_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  const std::string & expression_;
  const std::unordered_map<std::string, double> & variables_;
  std::string context_;
  std::size_t position_{};
};

double resolve_numeric_scalar(
  const YAML::Node & value_node, const std::string & expression_context, const std::string & conversion_message,
  const std::unordered_map<std::string, double> & variables)
{
  // 数値YAMLスカラーを優先し、裸の数値が文字列式として誤判定されることを防ぐ。
  try {
    return value_node.as<double>();
  } catch (const YAML::Exception &) {
  }

  try {
    const auto expression = value_node.as<std::string>();
    return ExpressionParser(unwrap_expansion_syntax(expression, expression_context), variables, expression_context).parse();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(conversion_message);
  }
}

double resolve_numeric_value(
  const YAML::Node & node, const std::string & key, const std::string & context,
  const std::unordered_map<std::string, double> & variables)
{
  if (!node[key]) {
    throw std::invalid_argument(context);
  }
  return resolve_numeric_scalar(node[key], context, context, variables);
}

int resolve_integer_value(
  const YAML::Node & node, const std::string & key, const std::string & context,
  const std::unordered_map<std::string, double> & variables)
{
  const auto value = resolve_numeric_value(node, key, context, variables);
  if (std::abs(value - std::round(value)) > 1e-9) {
    throw std::invalid_argument(context + " must resolve to an integer.");
  }
  return static_cast<int>(std::llround(value));
}

double resolve_step_duration(
  const YAML::Node & node, const std::string & path, const std::unordered_map<std::string, double> & variables)
{
  const auto sec_node = node["sec"];
  const auto duration_node = node["duration_sec"];
  if (sec_node && duration_node) {
    throw std::invalid_argument(path + " cannot use both sec and duration_sec.");
  }
  if (!sec_node && !duration_node) {
    throw std::invalid_argument(path + ".sec must be numeric.");
  }

  const auto key = sec_node ? std::string("sec") : std::string("duration_sec");
  const auto duration = resolve_numeric_value(node, key, path + "." + key + " must be numeric.", variables);
  if (duration <= 0.0) {
    throw std::invalid_argument(path + "." + key + " must be > 0.");
  }
  return duration;
}

std::unordered_map<std::string, double> load_variables(const YAML::Node & data)
{
  std::unordered_map<std::string, double> variables;
  const auto vars_node = data["vars"];
  if (!vars_node) {
    return variables;
  }
  if (!vars_node.IsMap()) {
    throw std::invalid_argument("Scenario YAML 'vars' must be a mapping.");
  }

  // 変数定義は上から順に評価し、後続の式からだけ参照できる単純な依存関係に限定する。
  for (const auto & item : vars_node) {
    const auto name = item.first.as<std::string>();
    if (name.empty() || !is_identifier_start(name.front())) {
      throw std::invalid_argument("Scenario YAML variable name is invalid: " + name);
    }
    for (const auto character : name) {
      if (!is_identifier_character(character)) {
        throw std::invalid_argument("Scenario YAML variable name is invalid: " + name);
      }
    }

    const auto value_node = item.second;
    try {
      variables.emplace(name, value_node.as<double>());
      continue;
    } catch (const YAML::Exception &) {
    }

    try {
      const auto expression = value_node.as<std::string>();
      variables.emplace(
        name, ExpressionParser(unwrap_expansion_syntax(expression, "vars." + name), variables, "vars." + name).parse());
    } catch (const YAML::Exception &) {
      throw std::invalid_argument("vars." + name + " must be numeric or an arithmetic expression.");
    }
  }
  return variables;
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
  bool & stop_used, const std::unordered_map<std::string, double> & variables)
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
  const auto speed = resolve_numeric_value(
    raw_command, "speed",
    "steps[" + std::to_string(step_index) + "].commands[" + std::to_string(command_index) +
    "].speed must be numeric.", variables);
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

MoveToPoseMode parse_move_to_pose_mode(const YAML::Node & node, const std::string & path)
{
  if (!node["type"]) {
    return MoveToPoseMode::Absolute;
  }

  const auto mode = required_as<std::string>(node, "type", path + ".type must be abs or rel.");
  if (mode == "abs") {
    return MoveToPoseMode::Absolute;
  }
  if (mode == "rel") {
    return MoveToPoseMode::Relative;
  }
  throw std::invalid_argument(path + ".type must be abs or rel.");
}

MoveTarget parse_move_to_pose_position(
  const YAML::Node & node, const std::string & path, const std::unordered_map<std::string, double> & variables)
{
  const auto pos_node = node["pos"];
  if (!pos_node) {
    return MoveTarget{};
  }

  MoveTarget position;
  position.tolerance = kDefaultPositionTolerance;

  // map形式は明示的なキー指定を優先し、tolerance省略時だけ既定値を使う。
  if (pos_node.IsMap()) {
    position.x = resolve_numeric_value(pos_node, "x", path + ".pos.x must be numeric.", variables);
    position.y = resolve_numeric_value(pos_node, "y", path + ".pos.y must be numeric.", variables);
    if (pos_node["tolerance"]) {
      position.tolerance = resolve_numeric_value(pos_node, "tolerance", path + ".pos.tolerance must be numeric.", variables);
    }
  } else if (pos_node.IsSequence()) {
    if (pos_node.size() != 2 && pos_node.size() != 3) {
      throw std::invalid_argument(path + ".pos must be [x, y] or [x, y, tolerance].");
    }
    position.x = resolve_numeric_scalar(pos_node[0], path + ".pos[0]", path + ".pos[0] must be numeric.", variables);
    position.y = resolve_numeric_scalar(pos_node[1], path + ".pos[1]", path + ".pos[1] must be numeric.", variables);
    if (pos_node.size() == 3) {
      position.tolerance = resolve_numeric_scalar(
        pos_node[2], path + ".pos[2]", path + ".pos[2] must be numeric.", variables);
    }
  } else {
    throw std::invalid_argument(path + ".pos must be a mapping or a list [x, y, tolerance].");
  }

  if (position.tolerance <= 0.0) {
    throw std::invalid_argument(path + ".pos.tolerance must be > 0.");
  }
  return position;
}

PoseTarget parse_move_to_pose_yaw(
  const YAML::Node & node, const std::string & path, const std::unordered_map<std::string, double> & variables)
{
  const auto yaw_node = node["yaw"];
  if (!yaw_node) {
    return PoseTarget{};
  }

  PoseTarget yaw;
  double yaw_degrees = 0.0;
  double tolerance_degrees = kDefaultYawToleranceDegrees;

  // yawは短い単独数値も許可し、詳細指定が必要な場合だけmapまたはlistを使う。
  if (yaw_node.IsMap()) {
    yaw_degrees = resolve_numeric_value(yaw_node, "deg", path + ".yaw.deg must be numeric.", variables);
    if (yaw_node["tolerance"]) {
      tolerance_degrees = resolve_numeric_value(
        yaw_node, "tolerance", path + ".yaw.tolerance must be numeric.", variables);
    }
  } else if (yaw_node.IsSequence()) {
    if (yaw_node.size() != 1 && yaw_node.size() != 2) {
      throw std::invalid_argument(path + ".yaw must be [deg] or [deg, tolerance].");
    }
    yaw_degrees = resolve_numeric_scalar(yaw_node[0], path + ".yaw[0]", path + ".yaw[0] must be numeric.", variables);
    if (yaw_node.size() == 2) {
      tolerance_degrees = resolve_numeric_scalar(
        yaw_node[1], path + ".yaw[1]", path + ".yaw[1] must be numeric.", variables);
    }
  } else {
    yaw_degrees = resolve_numeric_scalar(yaw_node, path + ".yaw", path + ".yaw must be numeric.", variables);
  }

  if (tolerance_degrees <= 0.0) {
    throw std::invalid_argument(path + ".yaw.tolerance must be > 0.");
  }

  // YAMLではdegreeを使い、制御計算用の内部表現だけradianへ変換する。
  yaw.yaw = degrees_to_radians(yaw_degrees);
  yaw.tolerance = degrees_to_radians(tolerance_degrees);
  return yaw;
}

ScenarioStep parse_move_to_pose_step(
  const YAML::Node & raw_step, const std::string & path, const std::unordered_map<std::string, double> & variables)
{
  if (raw_step["commands"]) {
    throw std::invalid_argument(path + " cannot combine commands and move_to_pose.");
  }

  const auto move_node = raw_step["move_to_pose"];
  if (!move_node || !move_node.IsMap()) {
    throw std::invalid_argument(path + ".move_to_pose must be a mapping.");
  }

  ScenarioStep step{};
  step.type = ScenarioStepType::MoveToPose;
  step.duration_sec = resolve_step_duration(raw_step, path, variables);
  step.move_to_pose.mode = parse_move_to_pose_mode(move_node, path + ".move_to_pose");
  step.move_to_pose.has_position = static_cast<bool>(move_node["pos"]);
  step.move_to_pose.has_yaw = static_cast<bool>(move_node["yaw"]);
  if (!step.move_to_pose.has_position && !step.move_to_pose.has_yaw) {
    throw std::invalid_argument(path + ".move_to_pose must contain pos or yaw.");
  }
  step.move_to_pose.position = parse_move_to_pose_position(move_node, path + ".move_to_pose", variables);
  step.move_to_pose.yaw = parse_move_to_pose_yaw(move_node, path + ".move_to_pose", variables);
  return step;
}

std::string step_path(const std::vector<std::size_t> & indices)
{
  std::ostringstream stream;
  stream << "steps";
  for (const auto index : indices) {
    stream << "[" << index << "]";
  }
  return stream.str();
}

ScenarioStep parse_step(
  const YAML::Node & raw_step, const std::vector<std::size_t> & indices,
  const std::unordered_map<std::string, double> & variables)
{
  const auto path = step_path(indices);
  if (!raw_step.IsMap()) {
    throw std::invalid_argument(path + " must be a mapping.");
  }

  if (raw_step["move_to_pose"]) {
    return parse_move_to_pose_step(raw_step, path, variables);
  }

  ScenarioStep step{};
  step.type = ScenarioStepType::Velocity;
  step.duration_sec = resolve_step_duration(raw_step, path, variables);

  const auto commands = raw_step["commands"];
  if (!commands || !commands.IsSequence() || commands.size() == 0) {
    throw std::invalid_argument(path + " must contain commands or move_to_pose.");
  }

  // 各ステップはゼロ初期化から合成し、前ステップの速度持ち越しを防ぐ。
  bool stop_used = false;
  std::unordered_map<std::string, std::string> used_axes;
  for (std::size_t index = 0; index < commands.size(); ++index) {
    apply_command(step, used_axes, commands[index], indices.back(), index + 1, stop_used, variables);
  }
  if (stop_used && commands.size() != 1) {
    throw std::invalid_argument(path + " stop must be the only command in the step.");
  }

  return step;
}

void parse_step_entries(
  const YAML::Node & raw_steps, const std::vector<std::size_t> & parent_indices,
  std::vector<ScenarioStep> & output_steps, std::unordered_map<std::string, double> & variables)
{
  if (!raw_steps || !raw_steps.IsSequence()) {
    throw std::invalid_argument(step_path(parent_indices) + " must be a list.");
  }

  for (std::size_t index = 0; index < raw_steps.size(); ++index) {
    const auto raw_entry = raw_steps[index];
    auto current_indices = parent_indices;
    current_indices.push_back(index + 1);
    const auto path = step_path(current_indices);

    if (!raw_entry.IsMap()) {
      throw std::invalid_argument(path + " must be a mapping.");
    }

    // setステップは手続き的な変数更新だけを行い、以降のstep展開へ更新値を引き継ぐ。
    if (const auto set_node = raw_entry["set"]) {
      if (!set_node.IsMap()) {
        throw std::invalid_argument(path + ".set must be a mapping.");
      }
      for (const auto & item : set_node) {
        const auto name = item.first.as<std::string>();
        if (name.empty() || !is_identifier_start(name.front())) {
          throw std::invalid_argument(path + ".set has invalid variable name: " + name);
        }
        for (const auto character : name) {
          if (!is_identifier_character(character)) {
            throw std::invalid_argument(path + ".set has invalid variable name: " + name);
          }
        }
        variables[name] = resolve_numeric_scalar(
          item.second, path + ".set." + name, path + ".set." + name + " must be numeric or ${...}.", variables);
      }
      continue;
    }

    // repeatブロックはfor文相当として展開し、ネスト時も実行順が直列化されるようにする。
    if (const auto repeat_node = raw_entry["repeat"]) {
      if (!repeat_node.IsMap()) {
        throw std::invalid_argument(path + ".repeat must be a mapping.");
      }
      const auto count = resolve_integer_value(
        repeat_node, "count", path + ".repeat.count must be an integer.", variables);
      if (count <= 0) {
        throw std::invalid_argument(path + ".repeat.count must be > 0.");
      }
      const auto nested_steps = repeat_node["steps"];
      if (!nested_steps || !nested_steps.IsSequence() || nested_steps.size() == 0) {
        throw std::invalid_argument(path + ".repeat.steps must be a non-empty list.");
      }
      for (int repeat_index = 0; repeat_index < count; ++repeat_index) {
        parse_step_entries(nested_steps, current_indices, output_steps, variables);
      }
      continue;
    }

    output_steps.push_back(parse_step(raw_entry, current_indices, variables));
  }
}

}  // namespace

ScenarioDefinition load_scenario_file(const std::string & scenario_file)
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

  ScenarioDefinition scenario;
  if (data["loop"]) {
    throw std::invalid_argument("Top-level 'loop' has been removed. Use steps[].repeat instead.");
  }
  const auto variables = load_variables(data);
  auto mutable_variables = variables;

  // 起動直後に全ステップを検証し、走行中の設定エラーで停止しないようにする。
  scenario.steps.reserve(steps_node.size());
  parse_step_entries(steps_node, {}, scenario.steps, mutable_variables);
  if (scenario.steps.empty()) {
    throw std::invalid_argument("Scenario YAML must define at least one step.");
  }

  return scenario;
}

std::string format_step_command(const ScenarioStep & step)
{
  std::ostringstream stream;
  if (step.type == ScenarioStepType::MoveToPose) {
    const auto mode = step.move_to_pose.mode == MoveToPoseMode::Absolute ? "abs" : "rel";
    stream << std::fixed << std::setprecision(3) << "move_to_pose type=" << mode << ", pos=["
           << step.move_to_pose.position.x << ", " << step.move_to_pose.position.y
           << "], pos_tolerance=" << step.move_to_pose.position.tolerance
           << ", yaw=" << radians_to_degrees(step.move_to_pose.yaw.yaw)
           << "deg, yaw_tolerance=" << radians_to_degrees(step.move_to_pose.yaw.tolerance) << "deg"
           << std::setprecision(2) << ", duration=" << step.duration_sec << "s";
    return stream.str();
  }

  stream << std::fixed << std::setprecision(3) << "linear_x=" << step.linear_x << ", linear_y="
         << step.linear_y << ", angular_z=" << step.angular_z << std::setprecision(2)
         << ", duration=" << step.duration_sec << "s";
  return stream.str();
}

}  // namespace ai_ship_robot_gazebo
