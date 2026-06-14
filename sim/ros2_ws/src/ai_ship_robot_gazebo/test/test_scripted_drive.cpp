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
steps:
  - duration_sec: 2.5
    commands:
      - type: forward
        speed: 0.4
      - type: yaw_right
        speed: 0.2
  - duration_sec: 1.0
    commands:
      - type: stop
)");

  // YAMLから各軸の速度成分を合成し、既存Python版と同じ符号規約を維持する。
  const auto steps = ai_ship_robot_gazebo::load_scenario_file(path);
  ASSERT_EQ(steps.size(), 2U);
  EXPECT_DOUBLE_EQ(steps[0].duration_sec, 2.5);
  EXPECT_DOUBLE_EQ(steps[0].linear_x, 0.4);
  EXPECT_DOUBLE_EQ(steps[0].linear_y, 0.0);
  EXPECT_DOUBLE_EQ(steps[0].angular_z, -0.2);
  EXPECT_DOUBLE_EQ(steps[1].duration_sec, 1.0);
  EXPECT_DOUBLE_EQ(steps[1].linear_x, 0.0);
  EXPECT_DOUBLE_EQ(steps[1].linear_y, 0.0);
  EXPECT_DOUBLE_EQ(steps[1].angular_z, 0.0);
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
