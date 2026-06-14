# run_simulation rosbag記録開始タイミング修正計画

## 目的

simulation起動完了前にrosbag recordを開始してbag先頭に空白区間が入る問題を避ける。Gazebo、TF、LiDAR、IMUがpublishを始めてから記録とdrive scenarioを開始する。

## 方針

- `--record-bag` 指定時はsimulation launchをbackgroundで起動する。
- `/clock`、`/tf_static`、`/livox/lidar`、`/livox/imu` の初回メッセージを待ってからrosbag recordを開始する。
- `--drive-scenario` 指定時はrecord開始後にdriveを開始し、操作開始前の空白を減らす。
- `--record-bag` なしの通常起動は従来通りforeground起動を維持する。

## 検証

- `bash -n` で構文確認する。
- `--help` 表示に起動待ちの説明が反映されることを確認する。
- 短いscenarioでrecord開始後に自動終了できることを確認する。
