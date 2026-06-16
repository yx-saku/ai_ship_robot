# AgentCoding補助ツール

このディレクトリは、AgentCoding中に作成した開発・調査用ツールを置く場所です。

## 位置づけ
- 本番環境の起動・運用には使いません。
- `scripts/` 配下の運用スクリプトとは分けて管理します。
- コンテキストが変わった後でも、なぜ存在するか分かるように、このREADMEに用途を残します。

## ツール一覧

### `evaluate_lio_sam_bag.py`

LIO-SAM実行後のrosbag2を読み、odometryや初期推定値の簡易メトリクスをJSONで出力する調査用スクリプトです。

主な確認項目:
- `/lio_sam/mapping/odometry` の開始・終了姿勢、移動量、yaw変化量
- `/odometry/imu` の開始・終了姿勢、移動量、yaw変化量
- `/lio_sam/feature/cloud_info` の `initial_guess_x/y/z` の最大ジャンプ量
- `/tf` の指定parent/childをground truth相当として読んだ移動量

実行例:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 dev/agent_tools/evaluate_lio_sam_bag.py outputs/rosbag2/slam_xxx
```

元のsimulation bagをground truthとして別指定する例:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 dev/agent_tools/evaluate_lio_sam_bag.py \
  outputs/rosbag2/slam_xxx \
  --ground-truth-bag outputs/rosbag2/sim_xxx
```

注意:
- ROS 2 Humble環境のPythonで実行してください。
- rosbag2 topic構成が変わった場合は、引数でtopic名を上書きしてください。
