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
├── ros2_ws/
│   └── src/
│       └── ai_ship_robot_slam/        # 2台LiDAR統合、glim SLAM launch/config
├── ros2_ws_sim/
│   └── src/
│       ├── ai_ship_robot_description/ # ロボットURDF/Xacro、LiDAR配置
│       └── ai_ship_robot_gazebo/      # Gazebo world、launch、RViz設定、操作ノード
├── scripts/
│   ├── install/                   # 依存導入、cache補助、workspace build
│   └── app/                       # glim起動とシミュレーション補助script
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

開発環境ではDev Containerの利用を推奨します。本番環境やJetson上では、Dockerを使わずに `scripts/install/install.sh`、`scripts/install/install_third_party.sh`、`scripts/install/setup.sh` を実行する想定です。

## Dev Containerで使う

VS Codeでこのフォルダを開き、`Dev Containers: Reopen in Container` を実行します。

Docker image build時に `install.sh`、`install_third_party.sh`、`sim/install.sh`、`sim/install_third_party.sh` が順番に実行され、system依存とthird_party underlayを image へ導入します。

初回は、マウントされたworkspace側に生成物がまだないため、コンテナが開いた後に setup script を実行します。

```bash
bash scripts/install/setup.sh
bash scripts/install/sim/setup.sh
```

本番環境では次だけで必要な導入とbuildが完了します。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
```

開発環境でシミュレーションも使う場合は、続けてsimulation追加分を導入します。

```bash
bash scripts/install/sim/install.sh
bash scripts/install/sim/install_third_party.sh
bash scripts/install/sim/setup.sh
```

## インストール方針

- `scripts/install/install.sh`
  - 本番環境の標準導線です。
  - ROS 2 Humble と実機向け apt 依存を導入します。
- `scripts/install/sim/install.sh`
  - 開発環境でのみ追加実行します。
  - Gazebo/Livox simulation 向け apt 依存を導入します。
- `scripts/install/install_third_party.sh`
  - 実機とsimulationで共通の third_party を `/opt/ai_ship_robot` へ導入します。
  - `Livox-SDK2`、`livox_ros_driver2`、`gtsam`、`gtsam_points`、`glim`、`glim_ros2` を扱います。
- `scripts/install/sim/install_third_party.sh`
  - simulation専用の third_party を `/opt/ai_ship_robot` の同じROS underlayへ追加します。
  - `ros2_livox_simulation` を扱います。
- `scripts/install/setup.sh`
  - system側 third_party underlayを読み込み、`ros2_ws` のrosdep、build、shell設定更新を行います。
- `scripts/install/sim/setup.sh`
  - system側 third_party underlayを読み込み、`ros2_ws_sim` のrosdep、build、shell設定更新を行います。

`install*.sh` は system 依存と `/opt/ai_ship_robot` 配下の third_party 導入を扱い、`setup*.sh` が workspace本体のbuildを担当します。本番環境では `ros2_ws_sim/` を含めない構成を配備するだけで、同じ install/setup 導線を使えます。

glim用依存にはソースビルドする `GTSAM 4.2`、`gtsam_points`、`glim`、ROS 2ノードを提供する `glim_ros2`、PCL/TF/同期系ROSパッケージを含めます。GTSAMはシステムEigenを使い、gtsam_pointsはUbuntu 22.04のBoostに合わせた互換patchを適用します。CPU前提のため、glimはCUDA、viewer、OpenCV連携を無効化してビルドします。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
bash scripts/install/install.sh && bash scripts/install/sim/install.sh
bash scripts/install/install_third_party.sh && bash scripts/install/sim/install_third_party.sh
bash scripts/install/setup.sh && bash scripts/install/sim/setup.sh
```

### 外部repoの採用方針

外部repoは検証済みcommit SHAに固定し、default branchの更新で挙動が変わらないようにしています。更新する場合は、対象commitでビルド、起動、SLAM入力topic、RViz表示を確認してから `scripts/install` 配下の固定refを変更します。

| 用途 | repo | 採用理由 | 注意点 |
|---|---|---|---|
| Livox実機driver | `Livox-SDK/livox_ros_driver2` | Livox公式repo。MID-360対応が明記されている | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox SDK | `Livox-SDK/Livox-SDK2` | Livox公式SDK。driver buildに必要 | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox Gazebo simulation | `stm32f303ret6/livox_laser_simulation_RO2` | Gazebo ClassicでLivox系非反復走査を再現できる実装が限られるため、シミュレーション用途に限定して採用 | 非公式repo。実機向けの `install_third_party.sh` には含めない |
| factor graph | `borglab/gtsam` | GTSAM公式/有名ライブラリ。GLIM依存 | `4.2`相当の検証済みcommitに固定 |
| point cloud factors | `koide3/gtsam_points` | GLIM公式READMEで案内される関連repo | Ubuntu 22.04 Boost互換の最小patchを適用 |
| SLAM core | `koide3/glim` | GLIM本体。公式repo | CPU前提でCUDA/viewer/OpenCV連携を無効化 |
| ROS 2 node | `koide3/glim_ros2` | GLIM公式READMEで案内されるROS 2 wrapper。`glim_rosnode`を提供 | star数は少なめだが、GLIM公式構成の一部として採用 |

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
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
```

シミュレーションも使う場合は、続けて次を実行します。

```bash
bash scripts/install/sim/install.sh
bash scripts/install/sim/install_third_party.sh
bash scripts/install/sim/setup.sh
```

## シミュレーションを起動する

ホスト側でX11表示を許可します。

```bash
xhost +local:docker
```

コンテナまたはROS 2環境内で次を実行します。

```bash
bash scripts/app/sim/run_simulation.sh
```

主な起動オプションです。

```bash
bash scripts/app/sim/run_simulation.sh --no-rviz
bash scripts/app/sim/run_simulation.sh --no-gui
bash scripts/app/sim/run_simulation.sh --lite
bash scripts/app/sim/run_simulation.sh --build
```

- `--no-rviz`: RViz2を起動しない
- `--no-gui`: Gazebo Classic GUIを起動しない
- `--lite`: GUIなし、LiDAR ray数を低減して軽量起動する
- `--build`: 起動前にworkspace setupを実行する

## ロボットを操作する

シミュレーション起動後、別ターミナルで次を実行します。

```bash
bash scripts/app/sim/drive_robot.sh
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
bash scripts/app/sim/drive_robot.sh --linear-speed 0.15 --lateral-speed 0.15 --angular-speed 0.5
```

ROS topicを直接publishして操作することもできます。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.25}}" \
  -r 10
```

## glimでSLAMする

2台LiDAR点群と左Mid-360内蔵IMUを使ってglim SLAMを起動します。本番用入口の `scripts/app/run_glim_slam.sh` はシミュレーションを意識せず、glimだけを起動します。Gazeboと組み合わせる場合は `scripts/app/sim/run_glim_slam.sh` を使います。

GLIM本体とROS 2ノードは公式repoの実装をそのまま使います。`glim_rosnode` は1つの点群topicを入力にするため、本プロジェクトでは2台LiDARの `PointCloud2` をTFで `left_lidar_link` に変換し、`/slam/points` へ統合する小さなROS 2ノードだけを追加しています。

左Mid-360の内蔵IMU frameは `left_lidar_imu_link` です。Livox Mid-360 User Manual EN 15ページのIMU chip位置を使い、`left_lidar_link -> left_lidar_imu_link` は `x=0.011`, `y=0.02329`, `z=-0.04412` m、回転identityとして扱います。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/sim/install.sh
bash scripts/install/sim/install_third_party.sh
bash scripts/install/setup.sh
bash scripts/install/sim/setup.sh
bash scripts/app/sim/run_glim_slam.sh --lite
```

`GTSAMConfig.cmake` などのCMake依存が見つからない場合は、system側 third_party underlayを再ビルドします。

```bash
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
```

RVizなし、Gazebo GUIなしで起動する例です。

```bash
bash scripts/app/sim/run_glim_slam.sh --lite --no-rviz --no-gui
```

別端末でロボットを操作します。

```bash
bash scripts/app/sim/drive_robot.sh
```

主なSLAM関連topicです。

- 2台LiDAR統合点群: `/slam/points`
- SLAM用IMU入力: `/left_lidar/imu`
- 左LiDAR入力: `/left_lidar/points`
- 右LiDAR入力: `/right_lidar/points`
- SLAM用LiDAR frame: `left_lidar_link`
- SLAM用IMU frame: `left_lidar_imu_link`

確認コマンドです。

```bash
ros2 topic hz /slam/points
ros2 topic hz /left_lidar/imu
ros2 run tf2_ros tf2_echo left_lidar_link left_lidar_imu_link
ros2 run tf2_ros tf2_echo left_lidar_link right_lidar_link
ros2 run tf2_ros tf2_echo glim_odom base_footprint
```

実機や外部センサでglimだけを起動する場合は、シミュレーションを含めずにconfigを指定します。GLIMはJSON config directoryを読むため、IMU topicなどを変える場合は `config_ros.json` も対象topicに合わせます。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
bash scripts/app/run_glim_slam.sh \
  --left-points /left_lidar/points \
  --right-points /right_lidar/points \
  --config ros2_ws/src/ai_ship_robot_slam/config/glim_real
```

glimのROS 2パッケージ名や実行ファイル名が導入版と異なる場合は、起動時に差し替えます。

```bash
bash scripts/app/run_glim_slam.sh --glim-package glim_ros --glim-executable glim_rosnode
```

## LiDAR配置を変更する

LiDAR配置は起動オプションで切り替えます。

```bash
bash scripts/app/sim/run_simulation.sh --lidar-pattern lidar_pattern_dual_updown.urdf.xacro
```

利用できる配置パターンは実ファイルから表示できます。

```bash
bash scripts/app/sim/run_simulation.sh --help
```

LiDAR配置パターンは次のディレクトリにあります。

```text
ros2_ws_sim/src/ai_ship_robot_description/urdf/lidar/patterns/
```

LiDAR本体定義と共通設定は次にあります。

```text
ros2_ws_sim/src/ai_ship_robot_description/urdf/lidar/models/mid360_lidar.urdf.xacro
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

`Missing /opt/ros/humble/setup.bash`、`Missing ros2_ws/install/setup.bash`、または `Missing ros2_ws_sim/install/setup.bash` が表示される場合は、system導入またはworkspace setupが未完了です。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/setup.sh
```

apt mirrorを明示する場合は、次のように環境変数を指定します。

```bash
APT_PRIMARY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_SECURITY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_FORCE_IPV4=true \
APT_RETRIES=3 \
APT_HTTP_TIMEOUT=20 \
bash scripts/install/install.sh
```

Jetsonなどarm64環境で `ports.ubuntu.com` を使う場合は、必要に応じて `APT_PORTS_MIRROR` も指定します。

apt indexを強制更新する場合は次を使います。

```bash
APT_UPDATE_MAX_AGE_SECONDS=0 bash scripts/install/install.sh
```

rosdepの情報を強制更新する場合は次を使います。

```bash
ROSDEP_UPDATE_MAX_AGE_SECONDS=0 bash scripts/install/setup.sh
```
