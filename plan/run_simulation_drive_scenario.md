# run_simulation.sh シナリオ自動運転オプション実装計画

## 目的

`run_simulation.sh` からシミュレータ起動と同時に `drive_robot.sh --scenario` を起動できるようにし、別端末で自動運転コマンドを実行する手間を減らす。

## 方針

- `run_simulation.sh` に `--drive-scenario FILE` を追加する。
- `--drive-scenario` 指定時だけ、`ros2 launch ai_ship_robot_gazebo simulation.launch.py` をバックグラウンド起動する。
- シミュレーション起動直後に `sim/scripts/app/drive_robot.sh --scenario FILE` をバックグラウンド起動する。
- `--drive-start-delay SEC`、`--drive-loop`、`--drive-once` を追加し、`drive_robot.sh` の既存オプションへ転送する。
- 終了時は `drive_robot.sh`、`rosbag record`、`ros2 launch` の順に停止し、バックグラウンドプロセスを残さない。

## 検証

- `bash -n sim/scripts/app/run_simulation.sh` で構文を確認する。
- `bash sim/scripts/app/run_simulation.sh --help` でヘルプ表示を確認する。
- `drive_robot.sh` の既存 pytest を再実行し、シナリオ待機処理への影響がないことを確認する。
