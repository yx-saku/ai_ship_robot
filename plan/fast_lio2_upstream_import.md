# FAST-LIO2公式コード取り込み計画

## 目的

`1780473502280-hidden-squid.md`の方針に従い、公式`hku-mars/FAST_LIO`由来コードを固定commitで取り込み、以後のROS 2 Humble移植差分を追跡できる状態にする。

## 今回の作業範囲

- 公式FAST-LIOリポジトリを固定commitで取得する。
- 公式指定の`ikd-Tree` submodule実体も固定commitで取得する。
- 公式コードは未改修のまま`ros2_ws/src/ai_ship_robot_fast_lio2/upstream/FAST_LIO`へ配置する。
- 改修差分の追跡に不要な大容量docs、画像、GIF、PDF、ログ、PCD置き場は取り込まない。
- `UPSTREAM.md`に出所、commit、license注意点、今後の改修予定を記録する。
- ROS 1公式パッケージがROS 2ビルド対象として誤検出されないよう、`upstream/COLCON_IGNORE`を追加する。

## 固定するupstream

- FAST-LIO: `https://github.com/hku-mars/FAST_LIO.git`
- FAST-LIO commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- ikd-Tree submodule: `https://github.com/hku-mars/ikd-Tree.git`
- ikd-Tree commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`

## このコミットで行わないこと

- ROS 2 APIへの移植
- topic名の変更
- `ai_ship_robot_slam`連携
- FAST-LIO2のビルド設定変更
- Gazebo PointCloud2 fallback対応
- upstream docs/assetsの完全ミラーリング

## 次の作業

1. `ai_ship_robot_fast_lio2`のROS 2 package.xml/CMakeLists.txtを作成する。
2. 公式由来コードを移植用`src/`、`include/`へ段階的にコピーまたは移動する。
3. ROS 2 Humble向けに`rclcpp`、`tf2_ros`、`pcl_conversions`中心へ置換する。
4. `/livox/lidar`、`/livox/imu`入力と`/slam/*`出力に対応する。
