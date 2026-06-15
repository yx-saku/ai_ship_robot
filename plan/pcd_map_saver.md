# PCDマップ保存機能 実装計画

## 目的
- LIO-SAMの登録済み点群を蓄積し、必要なタイミングでPCDファイルとして保存する。

## 実装範囲
- `/lio_sam/mapping/cloud_registered` を購読するノードを追加する。
- 受信点群を `map` frame に変換してメモリ上に蓄積する。
- `std_srvs/srv/Trigger` の `/save_pcd_map` サービスで `{workspace}/cloud_map` 配下にPCDを保存する。
- 起動オプションは `--map` のみ追加する。

## 対象外
- PCDマップのトピック配信。
- サービス名の変更機能。
- 保存先の設定ファイル対応。
- rosbag replay時の追加エラーハンドリング。
