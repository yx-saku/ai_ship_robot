# rosbag prefixと再生引数変更計画

## 目的

シミュレーション収録bagとSLAM実行収録bagをprefixで明確に分け、再生時はprefixに基づく最新bagを省略指定できるようにする。

## 方針

- `run_simulation.sh` の既定prefixは現状通り `sim_` を維持する。
- `run_slam.sh` の既定record出力prefixを全経路で `slam_` に統一する。
- `run_slam.sh --bag-play` は値省略を許可し、省略時は `rosbag2/sim_*` のうち `metadata.yaml` を持つ最新ディレクトリを使う。
- `scripts/dev/replay_rosbag.sh` は `MODE [BAG_PATH] [OPTIONS]` 形式に変更し、modeは `sim` または `slam` に限定する。
- `scripts/dev/replay_rosbag.sh` でbag path省略時は、modeと同じprefixの最新bagを `rosbag2/` から選択する。

## 検証

- `bash -n` で変更したscriptの構文を確認する。
- `--help` 表示に新しい引数仕様が出ることを確認する。
- `--bag-play` 単独指定時に最新 `sim_*` bagが選択されることを、存在確認までのdryな実行経路で確認する。
- `replay_rosbag.sh sim` / `replay_rosbag.sh slam` の最新bag解決を確認する。
