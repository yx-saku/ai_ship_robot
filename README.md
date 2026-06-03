# AI Ship Robot LiDAR Simulation Workspace

Gazebo上でロボット本体とLiDAR配置を確認するための最小ワークスペースです。地図生成、実機起動、実機インストール用コードは含めていません。

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
│   └── ai_ship_robot_gazebo/      # Gazebo world、launch、RViz設定
└── scripts/
    ├── bootstrap_workspace.sh
    ├── drive_robot.sh
    ├── setup_workspace.sh
    └── run_simulation.sh
```

`cate_root_ca.crt`はDocker build時にコンテナ内の信頼済みCA証明書として登録されます。Dev Containerや`docker compose build`の前に、リポジトリ直下へ配置してください。

## Dev Container

VS Codeでこのフォルダを開き、`Dev Containers: Reopen in Container`を実行します。コンテナ作成後に依存解決とワークスペースビルドが1回だけ自動実行されます。

```bash
bash scripts/bootstrap_workspace.sh
```

手動で依存解決と再ビルドを行う場合も同じコマンドを使います。

新しいshellではROS 2とワークスペースのoverlayが自動で読み込まれます。手動で読み込み直す場合は次を使います。

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

- Gazebo
- ロボットモデル
- 選択したLivox MID-360相当LiDAR構成
- RViz2でのロボットモデル、TF、点群表示

RVizを起動しない場合は次を使います。

```bash
bash scripts/run_simulation.sh --no-rviz
```

Gazebo GUIを起動しない場合は次を使います。

```bash
bash scripts/run_simulation.sh --no-gui
```

低CPU環境向けにGazebo GUIを切り、LiDARの水平/垂直samplesを半分にする場合は次を使います。

```bash
bash scripts/run_simulation.sh --lite
```

起動前に`ros2_ws`をビルドする場合は次を使います。

```bash
bash scripts/run_simulation.sh --build
```

## LiDAR配置

LiDAR配置は`ros2_ws/src/ai_ship_robot_description/urdf/ai_ship_robot.urdf.xacro`のinclude行を直接編集して切り替えます。

```xml
<xacro:include filename="lidar_pattern_04_single_front_center_pitch45.urdf.xacro" />
```

選択できる配置パターンは次の4つです。

- `lidar_pattern_01_dual_front_corners_down.urdf.xacro`
- `lidar_pattern_02_dual_front_corners_down_yaw20.urdf.xacro`
- `lidar_pattern_03_dual_front_corners_down_yaw38.urdf.xacro`
- `lidar_pattern_04_single_front_center_pitch45.urdf.xacro`

LiDAR共通設定は`ros2_ws/src/ai_ship_robot_description/urdf/mid360_lidar.urdf.xacro`にあります。

現在のデフォルト設定は次の通りです。

- 本体サイズ: 720mm x 720mm
- LiDAR: 1台のMID-360相当モデル
- 点群: `/center_lidar/points`
- 2台構成時の点群: `/left_lidar/points`、`/right_lidar/points`

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
ros2 topic hz /left_lidar/points
ros2 topic hz /right_lidar/points
ros2 topic echo /odom --once
```

このワークスペースでは、3D SLAM検証向けに車輪物理ではなくGazeboのplanar moveで台車ロボットの平面移動を再現します。`cmd_vel`の`linear.x`、`linear.y`、`angular.z`で、全方位移動、円弧走行、その場回転を操作できます。

シミュレーション起動後、別端末で次を実行するとキーボードでロボットを操作できます。

```bash
bash scripts/drive_robot.sh
```

`drive_robot.sh`は`cmd_vel`をpublishする操作用スクリプトです。GazeboとLiDARは`run_simulation.sh`で起動します。

操作ノードを起動する前に`ros2_ws`をビルドする場合は次を使います。

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
bash scripts/bootstrap_workspace.sh
bash scripts/run_simulation.sh
```

停止します。

```bash
docker compose down
```

## トラブルシュート

GazeboやRVizが表示されない場合は、ホスト側でX11アクセスを許可してからコンテナを再起動してください。

```bash
xhost +local:docker
```

`Missing ros2_ws/install/setup.bash`と表示される場合は、先に依存解決とビルドを実行してください。

```bash
bash scripts/bootstrap_workspace.sh
```

Docker build時に`cate_root_ca.crt`が見つからない場合は、リポジトリ直下に社内/プロキシ用CA証明書を`cate_root_ca.crt`という名前で配置してください。
