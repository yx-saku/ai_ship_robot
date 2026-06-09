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
├── ros2_ws/src/
│   ├── ai_ship_robot_description/ # ロボットURDF/Xacro、LiDAR配置
│   ├── ai_ship_robot_gazebo/      # Gazebo world、launch、RViz設定、操作ノード
│   └── ai_ship_robot_slam/        # 2台LiDAR統合、glim SLAM launch/config
├── scripts/
│   ├── install/                   # 依存導入、cache補助、workspace build
│   └── app/                       # シミュレーション起動、キーボード操作
└── third_party/
    ├── ws/                        # セットアップ時にclone/buildされる外部ROS workspace
    └── vendor/                    # セットアップ時にclone/buildされる外部SDK
```

## 推奨環境

- Ubuntu 22.04相当
- ROS 2 Humble
- Gazebo Classic 11
- VS Code Dev Containers

開発環境ではDev Containerの利用を推奨します。本番環境やJetson上では、Dockerを使わずに `scripts/install/environment.sh` を直接実行する想定です。

## Dev Containerで使う

VS Codeでこのフォルダを開き、`Dev Containers: Reopen in Container` を実行します。

Docker image build時に `scripts/install/system_dependencies.sh` が実行され、ROS 2 Humbleとapt依存がimageへ導入されます。

初回は、コンテナが開いた後に外部repo取得とworkspace buildを実行します。

```bash
bash scripts/install/environment.sh --workspace-only
```

すべてをまとめて実行したい場合は次を使います。既定profileは開発フル環境の `dev` です。

```bash
bash scripts/install/environment.sh
```

## 目的別インストール

依存パッケージは目的別のprofileで切り替えます。

- `real`: 実機向け。基本パッケージとglim用依存を入れる
- `simulation`: シミュレーションのみ。基本パッケージとGazebo/Livox用依存を入れる
- `slam-sim`: シミュレーションでglim SLAMを行う。基本、シミュレーション用、glim用依存を入れる
- `dev`: 開発フル環境。`slam-sim` にテスト/静的確認用依存を加える

glim用依存にはソースビルドする `GTSAM 4.2`、`gtsam_points`、`glim`、ROS 2ノードを提供する `glim_ros2`、PCL/TF/同期系ROSパッケージを含めます。GTSAMはシステムEigenを使い、gtsam_pointsはUbuntu 22.04のBoostに合わせた互換patchを適用します。CPU前提のため、glimはCUDA、viewer、OpenCV連携を無効化してビルドします。

```bash
bash scripts/install/environment.sh --profile real
bash scripts/install/environment.sh --profile simulation
bash scripts/install/environment.sh --profile slam-sim
bash scripts/install/environment.sh --profile dev
```

依存だけ、またはworkspaceだけを処理する場合もprofileを指定できます。

```bash
bash scripts/install/environment.sh --system-only --profile slam-sim
bash scripts/install/environment.sh --workspace-only --profile slam-sim
```

profileではなくgroupを直接指定することもできます。指定できるgroupは `base`, `simulation`, `glim`, `dev-test` です。

```bash
bash scripts/install/environment.sh --groups base,glim
bash scripts/install/environment.sh --groups base,simulation,glim
```

### 外部repoの採用方針

外部repoは検証済みcommit SHAに固定し、default branchの更新で挙動が変わらないようにしています。更新する場合は、対象commitでビルド、起動、SLAM入力topic、RViz表示を確認してから `scripts/install/environment.sh` の固定refを変更します。

| 用途 | repo | 採用理由 | 注意点 |
|---|---|---|---|
| Livox実機driver | `Livox-SDK/livox_ros_driver2` | Livox公式repo。MID-360対応が明記されている | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox SDK | `Livox-SDK/Livox-SDK2` | Livox公式SDK。driver buildに必要 | ライセンス表記はGitHub API上 `NOASSERTION` のため、本番配布前に同梱ライセンス確認が必要 |
| Livox Gazebo simulation | `stm32f303ret6/livox_laser_simulation_RO2` | Gazebo ClassicでLivox系非反復走査を再現できる実装が限られるため、シミュレーション用途に限定して採用 | 非公式repo。実機profileには含めない |
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
bash scripts/install/environment.sh
```

ROS 2 Humbleとapt依存だけを先に入れる場合は次を使います。

```bash
bash scripts/install/environment.sh --system-only
```

外部repo取得、Livox SDK導入、workspace buildだけを実行する場合は次を使います。

```bash
bash scripts/install/environment.sh --workspace-only
```

## シミュレーションを起動する

ホスト側でX11表示を許可します。

```bash
xhost +local:docker
```

コンテナまたはROS 2環境内で次を実行します。

```bash
bash scripts/app/run_simulation.sh
```

主な起動オプションです。

```bash
bash scripts/app/run_simulation.sh --no-rviz
bash scripts/app/run_simulation.sh --no-gui
bash scripts/app/run_simulation.sh --lite
bash scripts/app/run_simulation.sh --build
```

- `--no-rviz`: RViz2を起動しない
- `--no-gui`: Gazebo Classic GUIを起動しない
- `--lite`: GUIなし、LiDAR ray数を低減して軽量起動する
- `--build`: 起動前にworkspace setupを実行する

## ロボットを操作する

シミュレーション起動後、別ターミナルで次を実行します。

```bash
bash scripts/app/drive_robot.sh
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
bash scripts/app/drive_robot.sh --linear-speed 0.15 --lateral-speed 0.15 --angular-speed 0.5
```

ROS topicを直接publishして操作することもできます。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.25}}" \
  -r 10
```

## glimでSLAMする

シミュレーション上の2台LiDAR点群とIMUを使ってglim SLAMを起動します。

GLIM本体とROS 2ノードは公式repoの実装をそのまま使います。`glim_rosnode` は1つの点群topicを入力にするため、本プロジェクトでは2台LiDARの `PointCloud2` をTFで `base_link` に変換し、`/slam/points` へ統合する小さなROS 2ノードだけを追加しています。

```bash
bash scripts/install/environment.sh --profile slam-sim
bash scripts/app/run_glim_slam.sh --with-sim --lite
```

`GTSAMConfig.cmake` などのCMake依存が見つからない場合は、GTSAM/gtsam_points/glimの外部workspaceを再ビルドします。

```bash
bash scripts/install/environment.sh --system-only --profile real
bash scripts/install/environment.sh --workspace-only --profile real
```

RVizなし、Gazebo GUIなしで起動する例です。

```bash
bash scripts/app/run_glim_slam.sh --with-sim --lite --no-rviz --no-gui
```

`--with-sim` では再起動時のGazebo entity名衝突を避けるため、未指定なら一意な `robot_name` を自動設定します。固定名で起動したい場合は `--robot-name NAME` を指定します。

別端末でロボットを操作します。

```bash
bash scripts/app/drive_robot.sh
```

主なSLAM関連topicです。

- 2台LiDAR統合点群: `/slam/points`
- IMU入力: `/imu/data`
- 左LiDAR入力: `/left_lidar/points`
- 右LiDAR入力: `/right_lidar/points`

確認コマンドです。

```bash
ros2 topic hz /slam/points
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link left_lidar_link
ros2 run tf2_ros tf2_echo base_link right_lidar_link
```

実機や外部センサでglimだけを起動する場合は、シミュレーションを含めずにtopicを指定します。

```bash
bash scripts/install/environment.sh --profile real
bash scripts/app/run_glim_slam.sh \
  --left-points /left_lidar/points \
  --right-points /right_lidar/points \
  --imu-topic /imu/data \
  --config ros2_ws/src/ai_ship_robot_slam/config/glim_real
```

glimのROS 2パッケージ名や実行ファイル名が導入版と異なる場合は、起動時に差し替えます。

```bash
bash scripts/app/run_glim_slam.sh --glim-package glim_ros --glim-executable glim_rosnode
```

## LiDAR配置を変更する

LiDAR配置は起動オプションで切り替えます。

```bash
bash scripts/app/run_simulation.sh --lidar-pattern lidar_pattern_dual_updown.urdf.xacro
```

利用できる配置パターンは実ファイルから表示できます。

```bash
bash scripts/app/run_simulation.sh --help
```

LiDAR配置パターンは次のディレクトリにあります。

```text
ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns/
```

LiDAR本体定義と共通設定は次にあります。

```text
ros2_ws/src/ai_ship_robot_description/urdf/lidar/models/mid360_lidar.urdf.xacro
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

`Missing /opt/ros/humble/setup.bash` または `Missing ros2_ws/install/setup.bash` が表示される場合は、環境構築またはworkspace buildが未完了です。

```bash
bash scripts/install/environment.sh
```

apt mirrorを明示する場合は、次のように環境変数を指定します。

```bash
APT_PRIMARY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_SECURITY_MIRROR=http://ftp.riken.jp/Linux/ubuntu \
APT_FORCE_IPV4=true \
APT_RETRIES=3 \
APT_HTTP_TIMEOUT=20 \
bash scripts/install/environment.sh
```

Jetsonなどarm64環境で `ports.ubuntu.com` を使う場合は、必要に応じて `APT_PORTS_MIRROR` も指定します。

apt indexを強制更新する場合は次を使います。

```bash
APT_UPDATE_MAX_AGE_SECONDS=0 bash scripts/install/environment.sh --system-only
```

rosdepの情報を強制更新する場合は次を使います。

```bash
ROSDEP_UPDATE_MAX_AGE_SECONDS=0 bash scripts/install/environment.sh --workspace-only
```
