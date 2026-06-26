#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string write_temp_scenario(const std::string & content)
{
  static int scenario_index = 0;
  const auto path = std::filesystem::temp_directory_path() /
    ("ai_ship_robot_scripted_drive_test_" + std::to_string(++scenario_index) + ".yaml");
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

TEST(ScriptedDriveCore, LoadsMoveToPoseSteps)
{
  const auto path = write_temp_scenario(R"(
vars:
  target_x: 1.0
  target_y: ${target_x + 1.0}
steps:
  - duration_sec: 2.0
    move_to_pose:
      type: abs
      pos:
        x: ${target_x}
        y: ${target_y}
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 2.0
  - duration_sec: 3.0
    move_to_pose:
      type: rel
      pos:
        x: 0.5
        y: -0.25
        tolerance: 0.1
      yaw:
        deg: -45.0
        tolerance: 3.0
)" );

  // move_to_poseはabs/relの基準種別と位置・yaw目標を1つのstepとして読み込む。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 2U);
  EXPECT_EQ(scenario.steps[0].type, ai_ship_robot_gazebo::ScenarioStepType::MoveToPose);
  EXPECT_EQ(scenario.steps[0].move_to_pose.mode, ai_ship_robot_gazebo::MoveToPoseMode::Absolute);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.tolerance, 0.05);
  EXPECT_NEAR(scenario.steps[0].move_to_pose.yaw.yaw, kPi / 2.0, 1e-12);
  EXPECT_NEAR(scenario.steps[0].move_to_pose.yaw.tolerance, 2.0 * kPi / 180.0, 1e-12);
  EXPECT_EQ(scenario.steps[1].move_to_pose.mode, ai_ship_robot_gazebo::MoveToPoseMode::Relative);
  EXPECT_DOUBLE_EQ(scenario.steps[1].move_to_pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(scenario.steps[1].move_to_pose.position.y, -0.25);
  EXPECT_NEAR(scenario.steps[1].move_to_pose.yaw.yaw, -kPi / 4.0, 1e-12);
}

TEST(ScriptedDriveCore, LoadsMoveToPoseShortForms)
{
  const auto path = write_temp_scenario(R"(
vars:
  target_x: 1.0
steps:
  - duration_sec: 1.0
    move_to_pose:
      type: abs
      pos: ["${target_x}", 1.0, 0.01]
      yaw: [90.0, 0.5]
  - duration_sec: 1.0
    move_to_pose:
      type: abs
      pos: [1.0, 1.0]
      yaw: [90.0]
  - duration_sec: 1.0
    move_to_pose:
      pos:
        x: 2.0
        y: 3.0
      yaw: 45.0
)" );

  // 省略記法では不足したtoleranceだけ既定値に補完し、数値と式を同じ規則で扱う。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 3U);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.y, 1.0);
  EXPECT_DOUBLE_EQ(scenario.steps[0].move_to_pose.position.tolerance, 0.01);
  EXPECT_NEAR(scenario.steps[0].move_to_pose.yaw.tolerance, 0.5 * kPi / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(scenario.steps[1].move_to_pose.position.tolerance, 0.05);
  EXPECT_NEAR(scenario.steps[1].move_to_pose.yaw.tolerance, 1.0 * kPi / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(scenario.steps[2].move_to_pose.position.tolerance, 0.05);
  EXPECT_EQ(scenario.steps[2].move_to_pose.mode, ai_ship_robot_gazebo::MoveToPoseMode::Absolute);
  EXPECT_NEAR(scenario.steps[2].move_to_pose.yaw.yaw, kPi / 4.0, 1e-12);
  EXPECT_NEAR(scenario.steps[2].move_to_pose.yaw.tolerance, 1.0 * kPi / 180.0, 1e-12);
}

TEST(ScriptedDriveCore, LoadsMoveToPoseWithPartialTargets)
{
  const auto path = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    move_to_pose:
      pos: [1.0, 1.0]
  - duration_sec: 1.0
    move_to_pose:
      yaw: 90.0
)" );

  // posまたはyawの片方だけ指定した場合は、未指定側を実行開始時の現在値維持として扱う。
  const auto scenario = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(scenario.steps.size(), 2U);
  EXPECT_TRUE(scenario.steps[0].move_to_pose.has_position);
  EXPECT_FALSE(scenario.steps[0].move_to_pose.has_yaw);
  EXPECT_FALSE(scenario.steps[1].move_to_pose.has_position);
  EXPECT_TRUE(scenario.steps[1].move_to_pose.has_yaw);
  EXPECT_EQ(scenario.steps[0].move_to_pose.mode, ai_ship_robot_gazebo::MoveToPoseMode::Absolute);
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

TEST(ScriptedDriveCore, RejectsMoveToPoseMixedWithCommands)
{
  const auto path = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    commands:
      - type: stop
    move_to_pose:
      type: abs
      pos:
        x: 1.0
        y: 1.0
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 1.0
)" );

  // commandsとmove_to_poseは同一step内で混在させず、操作責務を明確に保つ。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}

TEST(ScriptedDriveCore, RejectsStepWithoutCommandBody)
{
  const auto path = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
)" );

  // 通常stepはcommandsかmove_to_poseのどちらかを必ず持つ。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(path), std::invalid_argument);
}

TEST(ScriptedDriveCore, RejectsInvalidMoveToPoseTargets)
{
  const auto invalid_move = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    move_to_pose:
      type: abs
      pos:
        x: 1.0
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 1.0
)" );
  const auto invalid_pose = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    move_to_pose:
      type: world
      pos:
        x: 1.0
        y: 1.0
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 0.0
)" );
  const auto empty_target = write_temp_scenario(R"(
steps:
  - duration_sec: 1.0
    move_to_pose:
      type: abs
)" );

  // 目標指定の不足や許容誤差0は、制御開始前に設定エラーとして検出する。
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(invalid_move), std::invalid_argument);
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(invalid_pose), std::invalid_argument);
  EXPECT_THROW(ai_ship_robot_gazebo::load_scenario_file(empty_target), std::invalid_argument);
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
