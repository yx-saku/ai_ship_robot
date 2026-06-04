# AI Ship Robot LiDAR Simulation Workspace

Gazebo Classic 11上でロボット本体とLiDAR配置を確認するための最小ワークスペースです。`stm32f303ret6/livox_laser_simulation_RO2` のpluginを取り込み、Livox MID-360相当の非反復走査LiDARを再現します。地図生成、実機起動、実機インストール用コードは含めていません。

## 構成

```text
.
├── .devcontainer/
│   ├── devcontainer.json
│   └── Dockerfile
├── cate_root_ca.crt
├── docker-compose.yml
├── ros2_ws/src/
│   ├── ai_ship_robot_description/ # ロボットURDF/Xacro
│   └── ai_ship_robot_gazebo/      # Gazebo Classic world、launch、RViz設定
├── third_party_ws/                # setup時にcloneされる外部ROS workspace
├── third_party_vendor/            # setup時にcloneされる非ROS依存repo
└── scripts/
    ├── drive_robot.sh
    ├── install_environment.sh
    ├── setup_workspace.sh
    └── run_simulation.sh
```

`cate_root_ca.crt`はDocker build時にコンテナ内の信頼済みCA証明書として登録されます。Dev Containerや`docker compose build`の前に、リポジトリ直下へ配置してください。

## Dev Container

VS Codeでこのフォルダを開き、`Dev Containers: Reopen in Container`を実行します。Docker image build時にROS 2 Humbleと追加apt依存は導入されますが、コンテナ作成時に外部repo取得やworkspace buildは自動実行しません。

```bash
bash scripts/install_environment.sh --workspace-only
```

開発コンテナ内では、初回または再セットアップ時に上記を実行します。`livox_ros_driver2` は upstream の `build.sh humble` で、`ros2_ws` は通常の `colcon build` で順にビルドされます。

本番Jetsonなど、ROS 2 Humble導入済みの実機環境で追加依存導入からworkspace buildまでまとめて行う場合は次を使います。

```bash
bash scripts/install_environment.sh
```

新しいshellではROS 2とワークスペースのoverlayが自動で読み込まれます。手動で読み込み直す場合は次を使います。このスクリプトはclone、ビルド、インストールを行いません。

```bash
source scripts/setup_workspace.sh
```

## シミュレーション起動

ホスト側でX11表示を許可します。

```bash
xhost +local:docker
```

Dev Container内で次を実行します。

```bash
bash scripts/run_simulation.sh
```

起動内容は次の通りです。

- Gazebo Classic 11
- ロボットモデル
- `livox_laser_simulation_RO2` pluginによるMID-360相当LiDAR構成
- RViz2でのロボットモデル、TF、点群表示

RVizを起動しない場合は次を使います。

```bash
bash scripts/run_simulation.sh --no-rviz
```

Gazebo Classic GUIを起動しない場合は次を使います。

```bash
bash scripts/run_simulation.sh --no-gui
```

低CPU環境向けにGazebo Classic GUIを切り、Livox pluginのray数を1/4へ落とす場合は次を使います。

```bash
bash scripts/run_simulation.sh --lite
```

起動前に環境セットアップを明示的に実行する場合は次を使います。

```bash
bash scripts/run_simulation.sh --build
```

## LiDAR配置

LiDAR配置は`ros2_ws/src/ai_ship_robot_description/urdf/ai_ship_robot.urdf.xacro`のinclude行を直接編集して切り替えます。

```xml
<xacro:include filename="lidar_pattern_single.urdf.xacro" />
```

選択できる配置パターンは次の4つです。

- `lidar_pattern_single.urdf.xacro`
- `lidar_pattern_dual_out20.urdf.xacro`
- `lidar_pattern_dual_out38.urdf.xacro`
- `lidar_pattern_dual_updown.urdf.xacro`

LiDAR共通設定は`ros2_ws/src/ai_ship_robot_description/urdf/mid360_lidar.urdf.xacro`にあります。

現在のデフォルト設定は次の通りです。

- 本体サイズ: 720mm x 720mm
- LiDAR: 1台のMID-360相当モデル
- 点群: `/center_lidar/points`
- CustomMsg: `/center_lidar/custom`
- 2台構成時の点群: `/left_lidar/points`、`/right_lidar/points`
- 2台構成時のCustomMsg: `/left_lidar/custom`、`/right_lidar/custom`

主に調整する値です。

```xml
<xacro:property name="lidar_x" value="${base_length / 2.0 + mid360_depth / 2.0}" />
<xacro:property name="lidar_y" value="0.0" />
<xacro:property name="lidar_z" value="${base_center_z}" />
<xacro:property name="lidar_pitch" value="${45.0 * deg_to_rad}" />
```

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

## ロボット操作

このワークスペースでは、LiDAR配置検証向けに車輪物理ではなくGazebo Classicの平面移動pluginで台車ロボットの移動を再現します。`cmd_vel`の`linear.x`、`linear.y`、`angular.z`で、全方位移動、円弧走行、その場回転を操作できます。

シミュレーション起動後、別端末で次を実行するとキーボードでロボットを操作できます。

```bash
bash scripts/drive_robot.sh
```

`drive_robot.sh`は`cmd_vel`をpublishする操作用スクリプトです。Gazebo ClassicとLiDARは`run_simulation.sh`で起動します。

操作ノードを起動する前に環境セットアップを明示的に実行する場合は次を使います。

```bash
bash scripts/drive_robot.sh --build
```

操作キーは次の通りです。

- `w` / `i`: 前進成分をON/OFF
- `s` / `,`: 後進成分をON/OFF
- `j` / `l`: 左/右横移動成分をON/OFF
- `a` / `d`: yaw左/右回転成分をON/OFF
- `space` / `x` / `k`: 全停止
- `Q` / `Esc`: 終了

並進と旋回は順番にキーを押して組み合わせます。例えば、`w`で前進を開始してから`a`を押すと前進しながら左へ円弧走行します。`a`をもう一度押すとyaw成分だけが解除され、前進のみになります。

速度を変更する例です。

```bash
bash scripts/drive_robot.sh --linear-speed 0.15 --lateral-speed 0.15 --angular-speed 0.5
```

ROS topicを直接publishしてロボットを動かす例です。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.25}}" \
  -r 10
```

## docker compose

VS Code Dev Containerを使わない場合は、ホスト側で次を実行します。

```bash
xhost +local:docker
docker compose build
docker compose up -d
docker compose exec ai-ship-robot-dev bash
```

コンテナ内で次を実行します。

```bash
bash scripts/install_environment.sh --workspace-only
bash scripts/run_simulation.sh
```

停止します。

```bash
docker compose down
```

## トラブルシュート

Gazebo ClassicやRVizが表示されない場合は、ホスト側でX11アクセスを許可してからコンテナを再起動してください。

```bash
xhost +local:docker
```

`Missing ros2_ws/install/setup.bash`と表示される場合は、先に環境セットアップを実行してください。

```bash
bash scripts/install_environment.sh
```

Docker build時に`cate_root_ca.crt`が見つからない場合は、リポジトリ直下に社内/プロキシ用CA証明書を`cate_root_ca.crt`という名前で配置してください。
