# Mid-360 6軸IMU初期姿勢推定計画

## 目的

Mid-360内蔵6軸IMUを使う場合でも、水平開始に依存せずLIO-SAMへ初期roll/pitchを渡せるようにする。

## 方針

- LiDAR-IMU相対姿勢を表す `extrinsicRot` と `extrinsicRPY` はidentityに戻す。
- `imuRPYWeight` は `0.0` にし、初期化後に固定orientationへroll/pitchが引っ張られないようにする。
- `ai_ship_robot_slam` に6軸IMU初期姿勢推定ノードを追加する。
- 推定ノードは `/livox/imu` を購読し、静止判定を満たす初期サンプルだけでroll/pitchを決める。
- 推定後は決定したorientationを固定し、加速度と角速度はそのまま `/livox/imu_oriented` へpublishし続ける。
- UV-Lab版LIO-SAMはinstall時パッチで `q_from * extQRPY` を使うようにする。
- `lidar_link` hardcodeのTF修正は今回対象外にする。

## 検証項目

- `ai_ship_robot_slam` をbuildする。
- `run_lio_sam.sh` / `run_slam.sh` / `run_simulation.sh` の構文確認を行う。
- launch引数でraw IMUとLIO-SAM入力IMUが分かれていることを確認する。
- 既存bagを使い、`/livox/imu_oriented` のorientationがpublishされることを確認する。
