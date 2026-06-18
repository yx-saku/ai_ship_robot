#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
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

  ScenarioStep step{};
  step.duration_sec = resolve_numeric_value(
    raw_step, "duration_sec", path + ".duration_sec must be numeric.", variables);
  if (step.duration_sec <= 0.0) {
    throw std::invalid_argument(path + ".duration_sec must be > 0.");
  }

  const auto commands = raw_step["commands"];
  if (!commands || !commands.IsSequence() || commands.size() == 0) {
    throw std::invalid_argument(path + ".commands must be a non-empty list.");
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
  stream << std::fixed << std::setprecision(3) << "linear_x=" << step.linear_x << ", linear_y="
         << step.linear_y << ", angular_z=" << step.angular_z << std::setprecision(2)
         << ", duration=" << step.duration_sec << "s";
  return stream.str();
}

}  // namespace ai_ship_robot_gazebo
