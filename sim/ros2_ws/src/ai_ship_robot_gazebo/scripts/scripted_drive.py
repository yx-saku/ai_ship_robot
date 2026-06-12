#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node


COMMAND_AXES = {
    "forward": ("linear_x", 1.0),
    "backward": ("linear_x", -1.0),
    "left": ("linear_y", 1.0),
    "right": ("linear_y", -1.0),
    "yaw_left": ("angular_z", 1.0),
    "yaw_right": ("angular_z", -1.0),
}


@dataclass(frozen=True)
class ScenarioStep:
    duration_sec: float
    linear_x: float
    linear_y: float
    angular_z: float


class ScriptedDrive(Node):
    def __init__(self) -> None:
        super().__init__("scripted_drive")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("start_delay_sec", 0.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("publish_rate", 10.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.scenario_file = Path(str(self.get_parameter("scenario_file").value))
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.loop_enabled = bool(self.get_parameter("loop").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.steps = self._load_scenario(self.scenario_file)

    def _load_scenario(self, scenario_file: Path) -> list[ScenarioStep]:
        # 設定ファイルの構造を起動直後に厳密検証し、実行中エラーを避ける。
        if not scenario_file.is_file():
            raise ValueError(f"Scenario file not found: {scenario_file}")

        with scenario_file.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
            raise ValueError("Scenario YAML must contain a top-level 'steps' list.")

        steps: list[ScenarioStep] = []
        for index, raw_step in enumerate(data["steps"], start=1):
            steps.append(self._parse_step(index, raw_step))

        if not steps:
            raise ValueError("Scenario YAML must define at least one step.")
        return steps

    def _parse_step(self, index: int, raw_step: Any) -> ScenarioStep:
        # 各ステップは速度の持ち越しを防ぐため、毎回ゼロ初期化から合成する。
        if not isinstance(raw_step, dict):
            raise ValueError(f"steps[{index}] must be a mapping.")

        try:
            duration_sec = float(raw_step["duration_sec"])
        except KeyError as exc:
            raise ValueError(f"steps[{index}] is missing duration_sec.") from exc
        except (TypeError, ValueError) as exc:
            raise ValueError(f"steps[{index}].duration_sec must be numeric.") from exc

        if duration_sec <= 0.0:
            raise ValueError(f"steps[{index}].duration_sec must be > 0.")

        commands = raw_step.get("commands")
        if not isinstance(commands, list) or not commands:
            raise ValueError(f"steps[{index}].commands must be a non-empty list.")

        components = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}
        used_axes: dict[str, str] = {}
        stop_used = False

        for command_index, raw_command in enumerate(commands, start=1):
            self._apply_command(index, command_index, raw_command, components, used_axes)
            stop_used = stop_used or raw_command.get("type") == "stop"

        if stop_used and len(commands) != 1:
            raise ValueError(f"steps[{index}] stop must be the only command in the step.")

        return ScenarioStep(duration_sec=duration_sec, **components)

    def _apply_command(
        self,
        step_index: int,
        command_index: int,
        raw_command: Any,
        components: dict[str, float],
        used_axes: dict[str, str],
    ) -> None:
        # 同一軸の正負コマンド競合をここで止め、曖昧なTwist生成を禁止する。
        if not isinstance(raw_command, dict):
            raise ValueError(f"steps[{step_index}].commands[{command_index}] must be a mapping.")

        command_type = raw_command.get("type")
        if command_type == "stop":
            return
        if command_type not in COMMAND_AXES:
            raise ValueError(f"Unsupported command type at steps[{step_index}]: {command_type}")

        try:
            speed = float(raw_command["speed"])
        except KeyError as exc:
            raise ValueError(f"steps[{step_index}].commands[{command_index}] is missing speed.") from exc
        except (TypeError, ValueError) as exc:
            raise ValueError(f"steps[{step_index}].commands[{command_index}].speed must be numeric.") from exc

        if speed < 0.0:
            raise ValueError(f"steps[{step_index}].commands[{command_index}].speed must be >= 0.")

        axis_name, direction = COMMAND_AXES[command_type]
        if axis_name in used_axes:
            raise ValueError(
                f"steps[{step_index}] cannot combine {used_axes[axis_name]} and {command_type} on {axis_name}."
            )
        used_axes[axis_name] = command_type
        components[axis_name] = direction * speed

    def publish_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.publisher.publish(twist)

    def stop(self) -> None:
        self.publish_twist(0.0, 0.0, 0.0)

    def wait_for_sim_duration(self, duration_sec: float) -> bool:
        # 自動運転の待機はsim time基準で行い、Gazeboの進行に同期させる。
        start_time = None
        target_duration_ns = int(duration_sec * 1_000_000_000)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            current_time = self.get_clock().now()
            if current_time.nanoseconds == 0:
                continue
            if start_time is None:
                # /clock受信前の0時刻を除外し、起動時に観測した有効なsim timeを待機起点にする。
                start_time = current_time
                continue
            if (current_time - start_time).nanoseconds >= target_duration_ns:
                return True

        return False

    def format_step_command(self, step: ScenarioStep) -> str:
        # 実行中ログで確認しやすいよう、各軸の指令値を固定形式で整形する。
        return (
            f"linear_x={step.linear_x:.3f}, linear_y={step.linear_y:.3f}, "
            f"angular_z={step.angular_z:.3f}, duration={step.duration_sec:.2f}s"
        )

    def run(self) -> None:
        # publish_rateは送信周期として使い、ステップ継続時間そのものはsim timeで判定する。
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0.")
        if self.start_delay_sec > 0.0:
            self.get_logger().info(
                f"Waiting {self.start_delay_sec:.2f} sec before scenario start (sim time)"
            )
            if not self.wait_for_sim_duration(self.start_delay_sec):
                return

        period_sec = 1.0 / self.publish_rate
        keep_running = True
        while rclpy.ok() and keep_running:
            # ループ実行時も各周回の先頭を記録し、どのシナリオ周回か追跡できるようにする。
            self.get_logger().info("Starting scripted drive scenario")
            for step_index, step in enumerate(self.steps, start=1):
                deadline = self.get_clock().now().nanoseconds + int(step.duration_sec * 1_000_000_000)
                self.get_logger().info(f"Step {step_index}: {self.format_step_command(step)}")
                while rclpy.ok():
                    if not rclpy.ok():
                        return
                    self.publish_twist(step.linear_x, step.linear_y, step.angular_z)
                    rclpy.spin_once(self, timeout_sec=period_sec)
                    current_time = self.get_clock().now().nanoseconds
                    if current_time != 0 and current_time >= deadline:
                        break
                self.stop()
                self.get_logger().info(f"Step {step_index}: stop")
            keep_running = self.loop_enabled


def main() -> int:
    rclpy.init()
    node: ScriptedDrive | None = None

    try:
        # 例外時も停止publishを試みるため、Node生成後はfinallyで必ずstopを呼ぶ。
        node = ScriptedDrive()
        node.run()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"scripted_drive failed: {exc}", flush=True)
        return 1
    finally:
        if node is not None:
            try:
                node.stop()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
