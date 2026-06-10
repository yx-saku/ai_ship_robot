## 目的

third_party の外部repo取得とビルドを workspace 配下から system 側へ移し、Docker image build と直接インストールの両方で同じ配置を使えるようにする。

## 配置方針

```text
/opt/ai_ship_robot/
  ros_underlay/
    humble/
      third_party_ws/
        src/
        build/
        install/
        log/
  vendor/
    livox_sdk2/
      src/
      build/
      install/
```

- ROS package の third_party は `/opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws` を colcon workspace として扱う。
- 非ROSの `Livox-SDK2` は `/opt/ai_ship_robot/vendor/livox_sdk2` に分離し、同じ `/opt/ai_ship_robot` 配下へ集約する。
- source 順は `/opt/ros/${ROS_DISTRO}`、third_party underlay、プロジェクト workspace の順にする。

## スクリプト構成

- `scripts/install/install_third_party.sh`
  - 実機とシミュレーションで共通の third_party を導入する。
  - 対象は `Livox-SDK2`、`livox_ros_driver2`、`gtsam`、`gtsam_points`、`glim`、`glim_ros2`。
- `scripts/install/sim/install_third_party.sh`
  - シミュレーション専用の third_party を導入する。
  - 対象は `ros2_livox_simulation`。
- `scripts/install/setup.sh`
  - third_party の clone/build は行わず、system 側 underlay を source して `ros2_ws` だけを build する。
- `scripts/install/sim/setup.sh`
  - third_party の clone/build は行わず、system 側 underlay と `ros2_ws` を source して `ros2_ws_sim` だけを build する。

## Dockerfile導線

Dev Container image build では以下の順序で実行する。

```bash
bash scripts/install/install.sh
bash scripts/install/install_third_party.sh
bash scripts/install/sim/install.sh
bash scripts/install/sim/install_third_party.sh
```

コンテナ起動後は、workspace本体だけを build するために `setup.sh` と `sim/setup.sh` を実行する。

## 実装上の注意

- 既存の workspace 内 `third_party/ws/install` を source しない。
- 旧 `third_party` 経由の絶対パスを含む `ros2_ws` / `ros2_ws_sim` の生成物は再build対象として削除する。
- `Livox-SDK2` のライブラリパスは `/etc/ld.so.conf.d/ai_ship_robot.conf` に登録し、実行時に `livox_ros_driver2` から見えるようにする。
