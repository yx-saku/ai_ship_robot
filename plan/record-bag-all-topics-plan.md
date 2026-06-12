## 背景

- `sim/scripts/app/run_simulation.sh` と `aitran/scripts/app/run_slam.sh` の `--record-bag` は、現在は一部トピックを明示列挙して記録している。
- 今回は `--record-bag` 指定時に、実行中に見えている全トピックを記録する挙動へ揃える。

## 方針

1. 両スクリプトの rosbag 記録ヘルパーを、明示トピック列挙と全トピック記録の両方に対応させる。
2. `--record-bag` 利用時は、`--bag-topics` 未指定なら `-a` で全トピックを記録する。
3. `--bag-topics` 指定時は、その CSV で与えたトピックだけを記録する。
4. `--lidar-topics` / `--imu-topics` は両スクリプトから削除する。
5. README の説明を実装に合わせて更新する。

## 確認項目

- `bash -n sim/scripts/app/run_simulation.sh`
- `bash -n aitran/scripts/app/run_slam.sh`
