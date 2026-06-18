#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace
{

std::string write_temp_scenario(const std::string & content)
{
  const auto path = std::filesystem::temp_directory_path() / "ai_ship_robot_scripted_drive_test.yaml";
  std::ofstream stream(path);
  stream << content;
  return path.string();
}

}  // namespace

TEST(ScriptedDriveCore, LoadsCompositeScenario)
{
  const auto path = write_temp_scenario(R"(
vars:
  base_speed: 0.2
  fast_turn: ${base_speed * 2}
steps:
  - duration_sec: 2.5
    commands:
      - type: forward
        speed: ${base_speed + 0.2}
      - type: yaw_right
        speed: ${fast_turn / 2}
  - duration_sec: 1.0
    commands:
      - type: stop
)");

  // YAMLから各軸の速度成分を合成し、既存Python版と同じ符号規約を維持する。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 2U);
  EXPECT_DOUBLE_EQ(scenario.steps[0].duration_sec, 2.5);
  EXPECT_DOUBLE_EQ(scenario.steps[0].linear_x, 0.4);
  EXPECT_DOUBLE_EQ(scenario.steps[0].linear_y, 0.0);
  EXPECT_DOUBLE_EQ(scenario.steps[0].angular_z, -0.2);
  EXPECT_DOUBLE_EQ(scenario.steps[1].duration_sec, 1.0);
  EXPECT_DOUBLE_EQ(scenario.steps[1].linear_x, 0.0);
  EXPECT_DOUBLE_EQ(scenario.steps[1].linear_y, 0.0);
  EXPECT_DOUBLE_EQ(scenario.steps[1].angular_z, 0.0);
}

TEST(ScriptedDriveCore, RejectsConflictingAxis)
{
  const auto path = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    commands:
      - type: forward
        speed: 0.2
      - type: backward
        speed: 0.2
)");

  // 同一軸の正負指令を同時指定した場合は、実行前検証で失敗させる。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}

TEST(ScriptedDriveCore, ExpandsRepeatBlock)
{
  const auto path = write_temp_scenario(R"(
vars:
  repeats: ${1 + 2}
  lateral_speed: ${0.05 * 2}
steps:
  - repeat:
      count: ${repeats}
      steps:
        - duration_sec: ${1.0 + 0.5}
          commands:
            - type: left
              speed: ${lateral_speed}
        - duration_sec: 0.5
          commands:
            - type: stop
)" );

  // repeatブロックは入れ子step列をcount回だけ直列展開し、for文相当の記述を可能にする。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 6U);
  EXPECT_DOUBLE_EQ(scenario.steps[0].linear_y, 0.1);
  EXPECT_DOUBLE_EQ(scenario.steps[1].linear_x, 0.0);
  EXPECT_DOUBLE_EQ(scenario.steps[2].linear_y, 0.1);
  EXPECT_DOUBLE_EQ(scenario.steps[4].linear_y, 0.1);
}

TEST(ScriptedDriveCore, AppliesSetBetweenRepeatedBlocks)
{
  const auto path = write_temp_scenario(R"(
vars:
  stride: 99.0
steps:
  - set:
      stride: 1.0
  - repeat:
      count: 2
      steps:
        - duration_sec: ${stride}
          commands:
            - type: forward
              speed: 0.5
        - set:
            stride: ${stride + 1.0}
)" );

  // setステップは裸の数値と式の両方を解決し、後続のrepeat周回へ更新値を引き継ぐ。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 2U);
  EXPECT_DOUBLE_EQ(scenario.steps[0].duration_sec, 1.0);
  EXPECT_DOUBLE_EQ(scenario.steps[1].duration_sec, 2.0);
}

TEST(ScriptedDriveCore, RejectsUndefinedVariable)
{
  const auto path = write_temp_scenario(R"(
steps:
  - duration_sec: missing_duration
    commands:
      - type: stop
)" );

  // 未定義変数は実行時まで持ち越さず、読み込み時に明示エラーとして止める。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}

TEST(ScriptedDriveCore, RejectsBareExpressionString)
{
  const auto path = write_temp_scenario(R"(
vars:
  stride: 1.0
steps:
  - duration_sec: stride + 1.0
    commands:
      - type: stop
)" );

  // 文字列の式は${...}で明示し、リテラル文字列と数式の曖昧さを防ぐ。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}

TEST(ScriptedDriveCore, RejectsTopLevelLoop)
{
  const auto path = write_temp_scenario(R"(
loop: true
steps:
  - duration_sec: 1.0
    commands:
      - type: stop
)" );

  // 全体loopは廃止し、部分反復だけをrepeatブロックへ移して誤用を防ぐ。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}
