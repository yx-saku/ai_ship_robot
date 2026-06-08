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
│   └── ai_ship_robot_gazebo/      # Gazebo world、launch、RViz設定、操作ノード
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

すべてをまとめて実行したい場合は次を使います。

```bash
bash scripts/install/environment.sh
```

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

## LiDAR配置を変更する

LiDAR配置は次のXacroで切り替えます。

```text
ros2_ws/src/ai_ship_robot_description/urdf/ai_ship_robot.urdf.xacro
```

includeする配置ファイルを変更します。

```xml
<xacro:include filename="lidar_pattern_single.urdf.xacro" />
```

利用できる配置パターンです。

- `lidar_pattern_single.urdf.xacro`
- `lidar_pattern_dual_out20.urdf.xacro`
- `lidar_pattern_dual_out38.urdf.xacro`
- `lidar_pattern_dual_updown.urdf.xacro`

LiDAR共通設定は次にあります。

```text
ros2_ws/src/ai_ship_robot_description/urdf/mid360_lidar.urdf.xacro
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
