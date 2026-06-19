# AI Ship Robot Workspace

低背な台車ロボット向けに、Livox Mid-360 系 LiDAR を使った 3D SLAM 検証と Gazebo Classic シミュレーションを行う ROS 2 Humble ワークスペースです。

## 構成

```text
.
├── .devcontainer/   # 開発コンテナ設定
├── dev/             # 開発補助スクリプト
├── docs/            # 補足ドキュメント
├── install/         # 実機・共通 underlay 導入スクリプト
├── outputs/         # rosbag2、PCD 出力先
├── ros2_ws/         # SLAM 用 ROS 2 ワークスペース
├── scripts/         # 実行スクリプト
└── sim/             # Gazebo Classic シミュレーション環境
```

`ros2_ws/src` には現在 `ai_ship_robot_slam` パッケージのみがあります。`sim/ros2_ws/src` には `ai_ship_robot_description` と `ai_ship_robot_gazebo` があります。

## 主な機能

- `scripts/run_slam.sh` を入口にした LIO-SAM 起動
- Livox `CustomMsg` を前提にした Mid-360 向け SLAM 導線
- 複数 LiDAR の `CustomMsg` 融合ノード
- 6 軸 IMU 初期姿勢推定ノード
- hybrid登録点群を使った PCD map saver ノード
- Gazebo Classic 上の LiDAR 配置検証環境
- rosbag2 の収録と再生補助

## インストール

本番相当の最小導線です。

```bash
bash install/install.sh
bash install/install_third_party.sh
bash install/setup.sh
```

各スクリプトの役割は次のとおりです。

- `install/install.sh`
  - ROS 2 Humble と共通 apt 依存を導入します。
- `install/install_third_party.sh`
  - `/opt/ai_ship_robot` 配下に共通 third_party underlay を構築します。
  - `Livox-SDK2`、`livox_ros_driver2`、`gtsam`、`autorccar_interfaces`、`LIO-SAM_MID360_ROS2` を固定コミットで導入します。
- `install/setup.sh`
  - `ros2_ws` の rosdep と colcon build を実行し、環境読み込み設定を更新します。

third_party underlay はワークスペース外の `/opt/ai_ship_robot` 配下に構築されます。

```text
/opt/ai_ship_robot/
├── ros_underlay/humble/third_party_ws/
└── vendor/livox_sdk2/
```

## Dev Container

Dev Container を使う場合は、コンテナ起動後にワークスペース側の build を実行します。

```bash
bash install/setup.sh
```

シミュレーションも使う場合は `sim/install/setup.sh` も実行します。詳細は `sim/README.md` を参照してください。

## SLAM を起動する

標準の起動入口は `scripts/run_slam.sh` です。

```bash
bash scripts/run_slam.sh \
  --imu /lidar1/livox/imu \
  --config ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml
```

主な入力・関連トピックです。

- `/lidar1/livox/lidar`, `/lidar2/livox/lidar`: LIO-SAM `imageProjection` が直接購読する Livox `CustomMsg`
- `/lidar1/livox/imu`: 生 IMU
- `/lio_sam/mapping/cloud_registered_hybrid`: map saver 用 hybrid 登録点群
- `/lio_sam/mapping/odometry`: scan matching 後の global odometry
- `/lio_sam/mapping/path`: keyframe pose 列

`run_slam.sh` は次の用途を 1 本で扱います。

- 実機相当のトピックを受けて LIO-SAM を起動
- `--sim` によるシミュレーション同時起動
- `--bag-play` による rosbag 再生入力
- `--record-bag` による rosbag 記録
- `--map` による PCD map saver 有効化
- `--cloud-queue-drain-timeout` による bag 再生後の cloudQueue 待機上限指定

`--map` を付けた場合は、`pcd_map_saver_node` が全hybrid登録点群を odometry pose で keyframe 区間 submap へ合成し、保存時に最新の `/lio_sam/mapping/path` で map frame へ再配置して PCD を書き出します。hybrid点群はCloudInfo内で渡した近傍詳細点群と、SLAMに使用した粗い点群を合成したlocal frame点群です。submap 合成時の voxel leaf size は近傍raw点群と同じ既定 `0.01m` です。

CPU/DDS 負荷を抑えるため、`/lio_sam/deskew/cloud_deskewed`、`/lio_sam/feature/cloud_corner`、`/lio_sam/feature/cloud_surface`、`/lio_sam/mapping/map_global`、`/lio_sam/mapping/map_local`、`/lio_sam/mapping/trajectory`、`/lio_sam/mapping/cloud_registered`、`/lio_sam/mapping/cloud_registered_raw` は既定では publish しません。RViz 確認が必要な場合は `lio_sam_mid360.yaml` の対応する `publish*` パラメータを `true` にしてください。

rosbag を記録する例です。

```bash
bash scripts/run_slam.sh \
  --imu /lidar1/livox/imu \
  --record-bag
```

出力先の既定値は `outputs/rosbag2/slam_<timestamp>` です。

性能検証や通常保存では `/lio_sam/mapping/cloud_registered_hybrid,/lio_sam/mapping/odometry,/lio_sam/mapping/path,/clock,/tf_static` のように必要topicを明示してください。

記録済みシミュレーション bag を再生して LIO-SAM を起動する例です。

```bash
bash scripts/run_slam.sh \
  --bag-play outputs/rosbag2/sim_20260611_120000 \
  --bag-start-delay 10 \
  --imu /lidar1/livox/imu \
  --config ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360_sim.yaml
```

- bag 再生時は `use_sim_time=true` を強制します。
- `--config` 未指定で `--bag-play` した場合は、simulation向けの `lio_sam_mid360_sim.yaml` を既定で使います。
- `--bag-start-offset` は bag 内の再生開始位置、`--bag-start-delay` は実時間での再生開始待ち時間です。
- `--record-bag` を併用すると、再生中の topic と SLAM 出力を別の rosbag に同時記録できます。
- `--bag-loop` を付けない場合、bag 再生終了後は LIO-SAM の `cloudQueue` が空になるまで、または待機上限に達するまで待ってから SLAM を自動終了します。
- `--cloud-queue-drain-timeout SEC` を指定すると、`cloudQueue` 待機の最大秒数を制限できます。既定は `300` 秒で、`0` を指定した場合のみ timeout なしで待機します。

## rosbag 再生

`dev/replay_rosbag.sh` は rosbag と RViz2 をまとめて起動します。第 1 引数に `sim` または `slam` を指定します。

```bash
bash dev/replay_rosbag.sh sim outputs/rosbag2/sim_20260611_120000
bash dev/replay_rosbag.sh slam outputs/rosbag2/slam_20260611_120000
```

- `sim` の既定 RViz: `sim/ros2_ws/src/ai_ship_robot_gazebo/config/mid360_points.rviz`
- `slam` の既定 RViz: `ros2_ws/src/ai_ship_robot_slam/rviz/lio_sam.rviz`
- `--rate`、`--start-offset`、`--loop`、`--map` を指定できます。

## ai_ship_robot_slam パッケージ

`ros2_ws/src/ai_ship_robot_slam` には次のノードがあります。

- `slam_reference_lidar_static_tf_node`
- `livox_custommsg_to_pointcloud2_node`
- `pcd_map_saver_node`

主な設定ファイルです。

- `ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml`
- `ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360_sim.yaml`
- `ros2_ws/src/ai_ship_robot_slam/config/multi_lidar_fusion.yaml`
- `ros2_ws/src/ai_ship_robot_slam/launch/lio_sam.launch.py`
- `ros2_ws/src/ai_ship_robot_slam/rviz/lio_sam.rviz`

SLAM本体の性能・挙動に関する値は、用途に合う `lio_sam_*.yaml` に記述します。LiDAR fusionの入力topic、基準LiDAR topic、fusion後CustomMsg topic、LIO-SAM入力topicは `multi_lidar_fusion.yaml` に記述します。基準LiDAR frameは基準topicの `CustomMsg.header.frame_id` から自動取得します。CLIではrosbag再生、RViz、map保存などの実行制御だけを指定します。

## シミュレーション

Gazebo Classic 環境は `sim/` に分離されています。導入と起動手順は `sim/README.md` を参照してください。

## 補助スクリプト

- `dev/replay_rosbag.sh`: rosbag 再生と RViz 起動
- `dev/create_production_bundle.sh`: 配布用バンドル作成
- `dev/agent_tools/evaluate_lio_sam_bag.py`: LIO-SAM 結果の簡易評価

## 確認コマンド

```bash
ros2 topic hz /lidar1/livox/lidar
ros2 topic hz /lidar1/livox/imu
ros2 run tf2_ros tf2_echo lidar1_link lidar1_imu_link
```
