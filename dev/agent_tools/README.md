# AgentCoding補助ツール

このディレクトリには、開発中の調査や検証に使う補助ツールを置きます。本番運用スクリプトは含みません。

## 一覧

### `evaluate_lio_sam_bag.py`

LIO-SAM 実行後の rosbag2 を読み、簡易メトリクスを JSON で標準出力する調査用スクリプトです。

主な集計対象です。

- `/lio_sam/mapping/odometry`
- `/odometry/imu`
- `/lio_sam/feature/cloud_info`
- `/tf` 内の指定 parent/child 変換

主な出力内容です。

- 開始・終了姿勢
- 変位と経路長
- yaw 変化量
- `cloud_info` の初期推定ジャンプ量
- ground truth 相当 TF の移動量

## 実行例

ROS 2 Humble の Python 環境を読み込んでから実行します。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 dev/agent_tools/evaluate_lio_sam_bag.py outputs/rosbag2/slam_xxx
```

ground truth を別 bag から読む例です。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 dev/agent_tools/evaluate_lio_sam_bag.py \
  outputs/rosbag2/slam_xxx \
  --ground-truth-bag outputs/rosbag2/sim_xxx
```

## 主なオプション

- `--tf-topic`
- `--tf-parent`
- `--tf-child`
- `--lio-odom-topic`
- `--imu-odom-topic`
- `--cloud-info-topic`
- `--ground-truth-bag`

## 注意

- 入力は `metadata.yaml` を含む rosbag2 ディレクトリです
- `metadata.yaml` が無い場合は終了コード `2` で終了します
- topic 名が既定と異なる bag では各オプションで上書きしてください
- replay 後 bag では SLAM 側 TF と simulation 側 TF が混在するため、ground truth を厳密に取りたい場合は `--ground-truth-bag` で元 simulation bag を指定します
