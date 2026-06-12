# Mid-360シミュレーション補完topic計画

## 目的

Gazebo plugin由来のシミュレーションraw topicを残しつつ、LIO-SAMが実機Livox driverに近いtopicを購読できるようにする。

## 方針

- シミュレーションraw topicは `/left_lidar/custom` と `/left_lidar/imu` のまま維持する。
- sim側にMid-360補完ノードを追加し、補完後topicを `/livox/lidar` と `/livox/imu` へpublishする。
- 点群はGazebo pluginで欠落している `line` をLivox scan patternから補完する。
- IMUのlinear accelerationはUV-Lab版LIO-SAM_MID360_ROS2の前提に合わせ、m/s^2からG単位へ変換する。
- LIO-SAM本体は `/livox/lidar` と、6軸初期姿勢推定後の `/livox/imu_oriented` を既定入力にする。
- 既存fusion nodeのline補完は既定無効にし、シミュレーション固有補正はsim側へ寄せる。

## 実装方針

- `ai_ship_robot_gazebo` に `mid360_sim_adapter.py` を追加する。
- raw `CustomMsg` は `/left_lidar/custom` で購読し、補完後を `/livox/lidar` にpublishする。
- raw `Imu` は `/left_lidar/imu` で購読し、G単位変換後を `/livox/imu` にpublishする。
- 補完後topicは別topicとしてpublishし、raw topicはシミュレーション検証用として残す。
- SLAM launchの既定入力は `/livox/lidar` と `/livox/imu_oriented` に変更する。
- 既存fusion nodeのscan pattern由来line補完は既定無効にし、シミュレーション固有補正はsim側adapterへ寄せる。

## 検証項目

- `ai_ship_robot_gazebo` と `ai_ship_robot_slam` をbuildする。
- launch引数で `/livox/lidar`、raw IMU `/livox/imu`、LIO-SAM入力IMU `/livox/imu_oriented` が既定になっていることを確認する。
- Python補完ノードの構文確認を行う。
- 既存シミュレーションbagを短時間再生し、`/livox/lidar` と `/livox/imu` がpublishされることを確認する。
