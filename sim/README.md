# Simulation Workspace

`sim/` は Gazebo Classic 11 上で LiDAR 配置、点群取得、走行シナリオ、SLAM 入力を検証するためのシミュレーション環境です。

## 構成

```text
sim/
├── config/drive_scenarios/  # 自動走行シナリオ YAML
├── install/                 # simulation 用導入スクリプト
├── ros2_ws/src/
│   ├── ai_ship_robot_description/  # URDF/Xacro、LiDAR 配置
│   └── ai_ship_robot_gazebo/       # Gazebo launch、adapter、操作ノード
└── scripts/
    ├── drive_robot.sh
    └── run_simulation.sh
```

## 導入

ルート側セットアップ完了後に次を実行します。

```bash
bash sim/install/install.sh
bash sim/install/install_third_party.sh
bash sim/install/setup.sh
```

各スクリプトの役割は次のとおりです。

- `sim/install/install.sh`
  - Gazebo Classic、RViz2、simulation 用 apt 依存を導入します。
- `sim/install/install_third_party.sh`
  - `ros2_livox_simulation` を `/opt/ai_ship_robot` 配下の underlay へ導入します。
- `sim/install/setup.sh`
  - `sim/ros2_ws` を build し、環境読み込み設定を更新します。

Dev Container 利用時も、初回は次を実行します。

```bash
bash sim/install/setup.sh
```

## 起動

```bash
bash sim/scripts/run_simulation.sh
```

主なオプションです。

```bash
bash sim/scripts/run_simulation.sh --no-rviz
bash sim/scripts/run_simulation.sh --no-gui
bash sim/scripts/run_simulation.sh --lite
bash sim/scripts/run_simulation.sh --half-resolution
bash sim/scripts/run_simulation.sh --real-time-factor 0.2
bash sim/scripts/run_simulation.sh --lidar-pattern lidar_pattern_dual_out38.urdf.xacro
bash sim/scripts/run_simulation.sh --drive-scenario sim/config/drive_scenarios/around_world.yaml
```

- `--build`: 起動前に `sim/install/setup.sh` を実行
- `--lite`: GUI を無効化し、LiDAR 解像度を既定で quarter に低減
- `--quarter-resolution` / `--half-resolution` / `--full-resolution`: LiDAR サンプル密度切替
- `--real-time-factor`: 一時 world を生成して Gazebo の sim 時間速度を変更
- `--lidar-pattern`: `lidar_pattern_*.urdf.xacro` から LiDAR 配置を選択
- `--drive-scenario`: シミュレーション起動後に自動走行を開始
- `--record-bag`: readiness 完了後に rosbag2 を記録
- 自動走行の繰り返しはシナリオ YAML の `steps[].repeat` で指定

既定 world は `sim/ros2_ws/src/ai_ship_robot_gazebo/worlds/shipyard_indoor_100x50.world` です。

- `shipyard_indoor_100x50.world`
  - 原点中心 `100m x 50m` の造船所内空間です
  - 原点付近は robot の安全スポーン帯、中央は長い主通路、北側/南側/東側に評価ゾーンがあります
- `lidar_placement.world`
  - 床欠損や溝の回帰確認用 world として継続利用できます

旧 world を明示指定する例です。

```bash
bash sim/scripts/run_simulation.sh \
  --world sim/ros2_ws/src/ai_ship_robot_gazebo/worlds/lidar_placement.world
```

## 自動走行

シナリオを使う例です。

```bash
bash sim/scripts/run_simulation.sh \
  --drive-scenario sim/config/drive_scenarios/around_world.yaml \
  --drive-start-delay 5.0
```

- `--drive-start-delay` は sim time 基準で待機します
- シナリオ内の一部繰り返しは YAML の `repeat.count` / `repeat.steps` で指定します
- シナリオ完了時は既定で 5 秒後にシミュレーションを自動終了します
- `--no-auto-exit` で自動終了を抑制できます
- 既存シナリオは旧 world 基準で調整された可能性があるため、新既定 world では手動操作または短いシナリオから確認してください

## rosbag 記録

```bash
bash sim/scripts/run_simulation.sh --record-bag
```

- 既定出力先は `outputs/rosbag2/sim_<timestamp>` です
- トピック未指定時は `/lidarN/livox/lidar`、`/lidarN/livox/imu`、`/tf`、`/tf_static` を記録します
- `--bag-topics /cmd_vel,/lidar1/livox/lidar` のように絞り込みできます
- 全トピックを記録したい場合は `--bag-all-topics` を付けます
- 記録開始前に `/clock`、`/tf_static`、`/lidar1/livox/lidar`、`/lidar1/livox/imu`、`/odom`、`/cmd_vel` subscriber を確認します

## 手動操作

シミュレーション起動後、別ターミナルで実行します。

```bash
bash sim/scripts/drive_robot.sh
```

キー操作です。

- `w` / `i`: 前進成分の切替
- `s` / `,`: 後進成分の切替
- `j` / `l`: 左右移動成分の切替
- `a` / `d`: yaw 回転成分の切替
- `space` / `x` / `k`: 全停止
- `Q` / `Esc`: 終了

速度を指定する例です。

```bash
bash sim/scripts/drive_robot.sh --linear-speed 1.0 --lateral-speed 0.8 --angular-speed 0.5
```

- 手動操縦の既定値は最大並進速度 `84 m/min = 1.4 m/s`、最大回転速度 `50 deg/s = 0.873 rad/s` です
- `w` と `j` のような斜め入力では、`sqrt(vx^2 + vy^2) <= 1.4` を満たすよう publish 直前に正規化されます

シナリオ単体実行の例です。

```bash
bash sim/scripts/drive_robot.sh \
  --scenario sim/config/drive_scenarios/around_world.yaml \
  --start-delay 1.0
```

- YAML の通常 step は `duration_sec` と `commands` または `move_to_pose` で記述します
- `move_to_pose.type: abs` は `/odom` 原点基準の絶対位置・絶対 yaw へ移動します
- `move_to_pose.type: rel` は step 開始時の位置・yaw を基準にした相対移動です
- `move_to_pose.type` を省略した場合は `abs` として扱います
- `move_to_pose.yaw.deg` と `move_to_pose.yaw.tolerance` は degree 単位です
- `move_to_pose.pos` は `[x, y]` / `[x, y, tolerance]`、`yaw` は数値 / `[deg]` / `[deg, tolerance]` でも指定できます
- `move_to_pose.pos` または `yaw` を省略した場合、未指定側は step 開始時の現在値を維持します
- `move_to_pose` の `duration_sec` は速度算出用の目標到達時間であり、超過しても timeout にはしません
- シナリオの繰り返しは `--loop` ではなく YAML の `repeat` ブロックで指定します
- シナリオ値が速度上限を超えた場合は、manual と同じ制約で補正され、warning ログが1回出ます

```yaml
steps:
  - duration_sec: 2.0
    commands:
      - type: forward
        speed: 0.2

  - duration_sec: 5.0
    move_to_pose:
      type: abs
      pos:
        x: 1.0
        y: 2.0
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 0.5

  - duration_sec: 5.0
    move_to_pose:
      type: rel
      pos: [1.0, 0.0]
      yaw: 90.0
```

## LiDAR 配置

配置ファイルの実体は次にあります。

```text
sim/ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns/
```

LiDAR 本体モデルは `sim/ros2_ws/src/ai_ship_robot_description/urdf/lidar/models/mid360_lidar.urdf.xacro` です。Gazebo 上では Livox plugin により `/${prefix}/custom` と `/${prefix}/imu` を出力し、PointCloud2 は既定では publish しません。

## 主なトピック

- `/cmd_vel`: 移動指令
- `/odom`: Gazebo planar move のオドメトリ
- `/imu/data`: ベース IMU
- `/lidar1/custom`, `/lidar2/custom`: 生 Livox `CustomMsg`
- `/lidar1/livox/lidar/points`, `/lidar2/livox/lidar/points`: RViz 用 bridge が生成する PointCloud2
- `/lidar1/imu`, `/lidar2/imu`: LiDAR 搭載 IMU
- `/lidar1/livox/lidar`, `/lidar2/livox/lidar`: `mid360_sim_adapter` で整形し、SLAM が直接購読する Livox `CustomMsg`
- `/lidar1/livox/imu`, `/lidar2/livox/imu`: `mid360_sim_adapter` で整形したLiDAR IMU

既定では `simulation.launch.py` が `/lidar1/custom` と `/lidar1/imu` を受けて `/lidar1/livox/lidar` と `/lidar1/livox/imu` を publish します。

`run_simulation.sh` は RViz 表示時だけ bridge を起動し、`/lidarN/livox/lidar` から `/lidarN/livox/lidar/points` を生成します。RViz は `/lidarN/livox/lidar/points` を購読します。

## 関連 launch

- `sim/ros2_ws/src/ai_ship_robot_gazebo/launch/simulation.launch.py`
  - Gazebo、robot_state_publisher、spawn、`mid360_sim_adapter`、RViz を起動します。
- `sim/ros2_ws/src/ai_ship_robot_gazebo/launch/sim_lio_sam.launch.py`
  - simulation と `ai_ship_robot_slam` の `lio_sam.launch.py` をまとめて起動します。

## トラブルシュート

`Missing /opt/ros/humble/setup.bash`、`Missing ros2_ws/install/setup.bash`、`Missing sim/ros2_ws/install/setup.bash` が出る場合は、セットアップ未完了です。

```bash
bash install/install.sh
bash install/install_third_party.sh
bash install/setup.sh
bash sim/install/install.sh
bash sim/install/install_third_party.sh
bash sim/install/setup.sh
```
