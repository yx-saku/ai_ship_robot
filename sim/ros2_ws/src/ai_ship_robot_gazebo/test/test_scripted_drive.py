from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


SCRIPTED_DRIVE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "scripted_drive.py"


def load_scripted_drive_module():
    # script配置の実行ファイルを、install前のソースから直接読み込んで単体検証する。
    spec = importlib.util.spec_from_file_location("scripted_drive", SCRIPTED_DRIVE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def __sub__(self, other: "FakeTime") -> FakeDuration:
        return FakeDuration(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self, samples: list[int]) -> None:
        self._samples = samples
        self.observed: list[int] = []

    def now(self) -> FakeTime:
        # 用意したclock列を順に返し、最後の値は反復上限として保持する。
        index = min(len(self.observed), len(self._samples) - 1)
        value = self._samples[index]
        self.observed.append(value)
        return FakeTime(value)


class FakeNode:
    def __init__(self, clock: FakeClock) -> None:
        self._clock = clock

    def get_clock(self) -> FakeClock:
        return self._clock


def test_wait_for_sim_duration_starts_at_first_valid_clock(monkeypatch):
    scripted_drive = load_scripted_drive_module()
    second = 1_000_000_000
    fake_clock = FakeClock([0, 10 * second, 14 * second + 999_000_000, 15 * second])
    fake_node = FakeNode(fake_clock)
    spin_timeouts: list[float] = []

    # /clock初回受信前の0を起点にせず、10秒時点から5秒進むまで待つことを確認する。
    monkeypatch.setattr(scripted_drive.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        scripted_drive.rclpy,
        "spin_once",
        lambda _node, timeout_sec: spin_timeouts.append(timeout_sec),
    )

    assert scripted_drive.ScriptedDrive.wait_for_sim_duration(fake_node, 5.0)
    assert fake_clock.observed == [0, 10 * second, 14 * second + 999_000_000, 15 * second]
    assert spin_timeouts
