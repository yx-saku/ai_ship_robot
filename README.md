# AI Ship Robot LiDAR Simulation Workspace

低背な台車ロボットにLiDARを搭載し、Gazebo Classic 11上でLiDAR配置、点群、ロボット移動を確認するためのROS 2 Humbleワークスペースです。

このワークスペースでは、Livox MID-360相当の非反復走査LiDARをシミュレーションし、ロボット周辺の点群取得と配置パターンの比較を行います。台車の移動は車輪機構を厳密に再現せず、全方位移動、円弧走行、その場回転を確認できる簡易移動モデルで扱います。

## 主な内容

- ROS 2 Humble workspace
- Gazebo Classic 11 simulation
- RViz2による点群、TF、ロボットモデル表示
- Livox MID-360相当LiDAR plugin
- 1台または2台LiDAR配置パターンの切り替え
- キーボードによるロボット操作
- 開発コンテナと直接インストールの両方で使える環境構築スクリプト

## ディレクトリ構成

```text
.
├── .devcontainer/                 # VS Code Dev Container設定
├── docker-compose.yml             # Dockerfile build検証用の最小Compose設定
├── aitran/
│   ├── ros2_ws/
│   │   └── src/
│   │       └── ai_ship_robot_slam/        # LIO-SAM連携、Mid-360点群adapter、launch/config
│   └── scripts/
│       ├── install/                       # 本番向け依存導入とworkspace build
│       └── app/
│           ├── run_lio_sam.sh
│           └── run_slam.sh
├── sim/
│   ├── ros2_ws/
│   │   └── src/
│   │       ├── ai_ship_robot_description/ # ロボットURDF/Xacro、LiDAR配置
│   │       └── ai_ship_robot_gazebo/      # Gazebo world、launch、RViz設定、操作ノード
│   └── scripts/
│       ├── install/                       # simulation向け依存導入とworkspace build
│       └── app/
│           ├── drive_robot.sh
│           └── run_simulation.sh
└── plan/                          # 実装方針メモ
```

third_party は workspace 配下ではなく、system 側の `/opt/ai_ship_robot` 配下へ導入します。

```text
/opt/ai_ship_robot/
├── ros_underlay/humble/third_party_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   └── log/
└── vendor/livox_sdk2/
    ├── src/
    ├── build/
    └── install/
```

## 推奨環境

- Ubuntu 22.04相当
- ROS 2 Humble
- Gazebo Classic 11
- VS Code Dev Containers

開発環境ではDev Containerの利用を推奨します。本番環境やJetson上では、Dockerを使わずに `aitran/scripts/install/install.sh`、`aitran/scripts/install/install_third_party.sh`、`aitran/scripts/install/setup.sh` を実行する想定です。

## Dev Containerで使う

VS Codeでこのフォルダを開き、`Dev Containers: Reopen in Container` を実行します。

Docker image build時に `aitran/scripts/install/install.sh`、`aitran/scripts/install/install_third_party.sh`、`sim/scripts/install/install.sh`、`sim/scripts/install/install_third_party.sh` が順番に実行され、system依存とthird_party underlayを image へ導入します。

初回は、マウントされたworkspace側に生成物がまだないため、コンテナが開いた後に setup script を実行します。

```bash
bash aitran/scripts/install/setup.sh
bash sim/scripts/install/setup.sh
```

本番環境では次だけで必要な導入とbuildが完了します。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
```

開発環境でシミュレーションも使う場合は、続けてsimulation追加分を導入します。

```bash
bash sim/scripts/install/install.sh
bash sim/scripts/install/install_third_party.sh
bash sim/scripts/install/setup.sh
```

## インストール方針

- `aitran/scripts/install/install.sh`
  - 本番環境の標準導線です。
  - ROS 2 Humble と実機向け apt 依存を導入します。
- `sim/scripts/install/install.sh`
  - 開発環境でのみ追加実行します。
  - Gazebo/Livox simulation 向け apt 依存を導入します。
- `aitran/scripts/install/install_third_party.sh`
  - 実機とsimulationで共通の third_party を `/opt/ai_ship_robot` へ導入します。
  - `Livox-SDK2`、`livox_ros_driver2`、`gtsam`、`LIO-SAM` を扱います。
- `sim/scripts/install/install_third_party.sh`
  - simulation専用の third_party を `/opt/ai_ship_robot` の同じROS underlayへ追加します。
  - `ros2_livox_simulation` を扱います。
- `aitran/scripts/install/setup.sh`
  - system側 third_party underlayを読み込み、`aitran/ros2_ws` のrosdep、build、shell設定更新を行います。
- `sim/scripts/install/setup.sh`
  - system側 third_party underlayを読み込み、`sim/ros2_ws` のrosdep、build、shell設定更新を行います。

`install*.sh` は system 依存と `/opt/ai_ship_robot` 配下の third_party 導入を扱い、`setup*.sh` が workspace本体のbuildを担当します。本番環境では `sim/` を含めない構成を配備するだけで、同じ install/setup 導線を使えます。

LIO-SAM用依存にはソースビルドする `GTSAM 4.2`、LIO-SAM本体、PCL/TF/同期系ROSパッケージを含めます。GTSAMはシステムEigenを使い、LIO-SAMはthird_party underlayに固定commitで導入します。Mid-360の点群はproject側adapterで `x,y,z,intensity,ring,time` 形式へ正規化します。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
bash aitran/scripts/install/install.sh && bash sim/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh && bash sim/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh && bash sim/scripts/install/setup.sh
```

### 外部repoの採用方針

外部repoは検証済みcommit SHAに固定し、default branchの更新で挙動が変わらないようにしています。更新する場合は、対象commitでビルド、起動、SLAM入力topic、RViz表示を確認してから `aitran/scripts/install` と `sim/scripts/install` 配下の固定refを変更します。

| 用途 | repo | 採用理由 | 注意点 |
|---|---|---|---|
| Livox実機driver | `Livox-SDK/livox_ros_driver2` | Livox公式repo。MID-360対応が明記されている | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox SDK | `Livox-SDK/Livox-SDK2` | Livox公式SDK。driver buildに必要 | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox Gazebo simulation | `stm32f303ret6/livox_laser_simulation_RO2` | Gazebo ClassicでLivox系非反復走査を再現できる実装が限られるため、シミュレーション用途に限定して採用 | 非公式repo。実機向けの `install_third_party.sh` には含めない |
| factor graph | `borglab/gtsam` | GTSAM公式/有名ライブラリ。LIO-SAM依存 | `4.2`相当の検証済みcommitに固定 |
| SLAM core | `TixiaoShan/LIO-SAM` | LiDAR-Inertial SLAMの実績がある有名repo。ROS 2 branchを利用 | 固定commitで導入し、Mid-360固有の点群差分はproject側adapterで吸収 |

セットアップ後、新しいターミナルではROS 2とビルド済みworkspace overlayが自動で読み込まれます。現在のターミナルへ反映する場合は次を実行します。

```bash
source ~/.bashrc
```

## Dockerfile buildを検証する

Dev Containerを開けない場合の切り分けとして、Dockerfileのbuildだけを確認できます。

```bash
docker compose build
```

`docker-compose.yml` はbuild検証用の最小構成です。開発環境の起動、volume、環境変数、X11設定はDev Container側で管理します。

## 直接インストールで使う

Dockerを使わない環境では、対象マシン上で次を実行します。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
```

シミュレーションも使う場合は、続けて次を実行します。

```bash
bash sim/scripts/install/install.sh
bash sim/scripts/install/install_third_party.sh
bash sim/scripts/install/setup.sh
```

## シミュレーションを起動する

ホスト側でX11表示を許可します。

```bash
xhost +local:docker
```

コンテナまたはROS 2環境内で次を実行します。

```bash
bash sim/scripts/app/run_simulation.sh
```

主な起動オプションです。

```bash
bash sim/scripts/app/run_simulation.sh --no-rviz
bash sim/scripts/app/run_simulation.sh --no-gui
bash sim/scripts/app/run_simulation.sh --lite
bash sim/scripts/app/run_simulation.sh --build
```

- `--no-rviz`: RViz2を起動しない
- `--no-gui`: Gazebo Classic GUIを起動しない
- `--lite`: GUIなし、LiDAR ray数を低減して軽量起動する
- `--build`: 起動前にworkspace setupを実行する

rosbagを記録する例です。

```bash
bash sim/scripts/app/run_simulation.sh --record-bag
```

- 記録対象の既定値: `/left_lidar/custom`, `/right_lidar/custom`, `/left_lidar/imu`, `/right_lidar/imu`, `/clock`, `/tf`, `/tf_static`
- `--bag-output` 未指定時は `rosbag2/sim_<timestamp>` に保存します
- 記録時刻は `ros2 bag record --use-sim-time` で `/clock` を基準にします
- 追加topicは `--bag-topics /cmd_vel,/scan_debug` のようにCSVで指定できます
- LiDAR/IMU記録対象の上書きは `--lidar-topics` と `--imu-topics` を使います

## ロボットを操作する

シミュレーション起動後、別ターミナルで次を実行します。

```bash
bash sim/scripts/app/drive_robot.sh
```

操作キーです。

- `w` / `i`: 前進成分をON/OFF
- `s` / `,`: 後進成分をON/OFF
- `j` / `l`: 左右移動成分をON/OFF
- `a` / `d`: yaw回転成分をON/OFF
- `space` / `x` / `k`: 全停止
- `Q` / `Esc`: 終了

速度を指定する例です。

```bash
bash sim/scripts/app/drive_robot.sh --linear-speed 0.15 --lateral-speed 0.15 --angular-speed 0.5
```

YAMLシナリオで自動運転する例です。

```bash
bash sim/scripts/app/drive_robot.sh \
  --auto \
  --scenario sim/config/drive_scenarios/arc_left.yaml \
  --start-delay 1.0
```

- `--loop` を付けるとシナリオを繰り返します
- ステップごとに速度成分はリセットされ、終了時には `Twist=0` をpublishします

ROS topicを直接publishして操作することもできます。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.25}}" \
  -r 10
```

## LIO-SAMでSLAMする

左Mid-360の点群と内蔵IMUを使ってLIO-SAMを起動します。本番用入口の `aitran/scripts/app/run_lio_sam.sh` はシミュレーションを意識せず、LIO-SAMだけを起動します。Gazeboと同時に起動する場合は `aitran/scripts/app/run_slam.sh --sim` を使います。

LIO-SAM本体はthird_party underlayの公式repo実装を使います。本プロジェクトでは、Livox `CustomMsg` を複数台分融合したうえで、Mid-360の点群をLIO-SAMが期待する `x,y,z,intensity,ring,time` フィールドへ正規化するROS 2導線を追加しています。rosbag の既定記録対象も `CustomMsg` を使います。

左Mid-360の内蔵IMU frameは `left_lidar_imu_link` です。Livox Mid-360 User Manual EN 15ページのIMU chip位置を使い、`left_lidar_link -> left_lidar_imu_link` は `x=0.011`, `y=0.02329`, `z=-0.04412` m、回転identityとして扱います。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash sim/scripts/install/install.sh
bash sim/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
bash sim/scripts/install/setup.sh
bash aitran/scripts/app/run_slam.sh --sim --lite
```

`GTSAMConfig.cmake` や `lio_sam` package が見つからない場合は、system側 third_party underlayを再ビルドします。

```bash
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
```

RVizなし、Gazebo GUIなしで起動する例です。

```bash
bash aitran/scripts/app/run_slam.sh --sim --lite --no-rviz --no-gui
```

シミュレーションSLAMと同時にrosbagを記録する例です。

```bash
bash aitran/scripts/app/run_slam.sh --sim --lite --record-bag
```

- `--bag-output` 未指定時は `rosbag2/sim_slam_<timestamp>` に保存します
- 記録時刻は `/clock` を基準にします
- `run_slam.sh --bag-play` では bag 内の `/tf_static` のみを再生し、`/tf` は再生しません

別端末でロボットを操作します。

```bash
bash sim/scripts/app/drive_robot.sh
```

主なSLAM関連topicです。

- LIO-SAM用変換後点群: `/left_lidar/lio_sam_points`
- SLAM用IMU入力: `/left_lidar/imu`
- 左LiDAR入力CustomMsg: `/left_lidar/custom`
- 融合入力CustomMsg: `/left_lidar/custom`, `/right_lidar/custom`
- SLAM用LiDAR frame: `left_lidar_link`
- SLAM用IMU frame: `left_lidar_imu_link`

確認コマンドです。

```bash
ros2 topic hz /left_lidar/custom
ros2 topic hz /left_lidar/imu
ros2 run tf2_ros tf2_echo left_lidar_link left_lidar_imu_link
ros2 run tf2_ros tf2_echo odom base_footprint
```

実機や外部センサでLIO-SAMだけを起動する場合は、シミュレーションを含めずにtopicやconfigを指定します。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
bash aitran/scripts/app/run_slam.sh \
  --points /left_lidar/custom \
  --imu /left_lidar/imu \
  --config aitran/ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml
```

実機相当のtopic入力を記録しながらLIO-SAMを起動する例です。

```bash
bash aitran/scripts/app/run_slam.sh \
  --points /left_lidar/custom \
  --imu /left_lidar/imu \
  --record-bag
```

- 実機相当入力の記録先既定値は `rosbag2/lio_sam_<timestamp>` です
- 記録時は `/tf` と `/tf_static` も含めます

記録済みbagを再生してGazeboなしでLIO-SAMを起動する例です。

```bash
bash aitran/scripts/app/run_slam.sh \
  --bag-play rosbag2/sim_20260611_120000 \
  --imu /left_lidar/imu \
  --input-points /left_lidar/custom,/right_lidar/custom
```

- bag再生時は `use_sim_time=true` を強制します

## rosbagの中身を確認する

開発用ツールとして、rosbag と RViz2 をまとめて起動するスクリプトを追加しています。

```bash
bash scripts/dev/replay_rosbag.sh rosbag2/sim_20260611_120000
```

- 既定の RViz 設定は `sim/ros2_ws/src/ai_ship_robot_gazebo/config/mid360_points.rviz` を使います
- `replay_rosbag.sh` は bag に含まれる `/tf` と `/tf_static` もそのまま再生します
- 再生速度は `--rate 0.5`、ループ再生は `--loop`、開始オフセットは `--start-offset 5` で指定できます
- 別の RViz 設定を使う場合は `--rviz-config path/to/file.rviz` を指定します

third_party側のLIO-SAM ROS 2パッケージ名が導入版と異なる場合は、起動時に差し替えます。

```bash
bash aitran/scripts/app/run_lio_sam.sh --lio-sam-package lio_sam
```

## LiDAR配置を変更する

LiDAR配置は起動オプションで切り替えます。

```bash
bash sim/scripts/app/run_simulation.sh --lidar-pattern lidar_pattern_dual_updown.urdf.xacro
```

利用できる配置パターンは実ファイルから表示できます。

```bash
bash sim/scripts/app/run_simulation.sh --help
```

LiDAR配置パターンは次のディレクトリにあります。

```text
sim/ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns/
```

LiDAR本体定義と共通設定は次にあります。

```text
sim/ros2_ws/src/ai_ship_robot_description/urdf/lidar/models/mid360_lidar.urdf.xacro
```

主な出力topicです。

- 1台構成点群: `/center_lidar/points`
- 1台構成CustomMsg: `/center_lidar/custom`
- 2台構成点群: `/left_lidar/points`, `/right_lidar/points`
- 2台構成CustomMsg: `/left_lidar/custom`, `/right_lidar/custom`

## 確認コマンド

```bash
ros2 topic list
ros2 topic hz /center_lidar/points
ros2 topic echo /center_lidar/custom --once
ros2 topic hz /left_lidar/points
ros2 topic hz /right_lidar/points
ros2 topic hz /imu/data
ros2 topic echo /odom --once
ros2 topic echo /clock --once
```

## トラブルシュート

Gazebo ClassicやRViz2が表示されない場合は、ホスト側でX11アクセスを許可してからコンテナを再起動します。

```bash
xhost +local:docker
```

`Missing /opt/ros/humble/setup.bash`、`Missing aitran/ros2_ws/install/setup.bash`、または `Missing sim/ros2_ws/install/setup.bash` が表示される場合は、system導入またはworkspace setupが未完了です。

```bash
bash aitran/scripts/install/install.sh
bash aitran/scripts/install/install_third_party.sh
bash aitran/scripts/install/setup.sh
bash sim/scripts/install/setup.sh
```

apt mirrorを明示する場合は、次のように環境変数を指定します。

```bash
APT_PRIMARY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_SECURITY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_FORCE_IPV4=true \
APT_RETRIES=3 \
APT_HTTP_TIMEOUT=20 \
bash aitran/scripts/install/install.sh
```

Jetsonなどarm64環境で `ports.ubuntu.com` を使う場合は、必要に応じて `APT_PORTS_MIRROR` も指定します。

apt indexを強制更新する場合は次を使います。

```bash
APT_UPDATE_MAX_AGE_SECONDS=0 bash aitran/scripts/install/install.sh
```

rosdepの情報を強制更新する場合は次を使います。

```bash
ROSDEP_UPDATE_MAX_AGE_SECONDS=0 bash aitran/scripts/install/setup.sh
```
