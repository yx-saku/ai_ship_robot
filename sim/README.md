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

既定 world は `sim/ros2_ws/src/ai_ship_robot_gazebo/worlds/lidar_placement.world` です。

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
bash sim/scripts/drive_robot.sh --linear-speed 0.15 --lateral-speed 0.15 --angular-speed 0.5
```

シナリオ単体実行の例です。

```bash
bash sim/scripts/drive_robot.sh \
  --scenario sim/config/drive_scenarios/around_world.yaml \
  --start-delay 1.0
```

- シナリオの繰り返しは `--loop` ではなく YAML の `repeat` ブロックで指定します

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
- `/lidar1/livox/lidar`, `/lidar1/livox/imu`: `mid360_sim_adapter` で整形したfusion入力
- `/fused/livox/lidar`: SLAM が購読するfusion後の Livox `CustomMsg`

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
