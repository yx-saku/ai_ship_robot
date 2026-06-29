# LIO-SAM 改修内容詳細

更新日: 2026-06-16

## 目的

この文書は、`UV-Lab/LIO-SAM_MID360_ROS2` に対して本プロジェクトで加えている改修内容を、現行実装に基づいて洗い出したものである。

主な目的は以下である。

- Mid-360内蔵6軸IMUでLIO-SAMを起動できるようにする。
- 実機Livox入力とGazebo simulation入力の単位差、時刻差、topic差を吸収する。
- rosbag再生や高頻度入力でCloudInfoやLiDAR scanが落ちにくい構成にする。
- LIO-SAMの推定frameとロボットURDFの実frameを分離し、TF競合を避ける。
- third_party本体への変更をoverrideファイルとして管理し、固定commitから再現可能にする。

## 改修点の簡易一覧

詳細に入る前に、LIO-SAM本体へ加えている主な改修を簡単に整理する。

| 改修点 | 概要 |
|---|---|
| 6軸IMU対応 | 起動直後の静止IMUからroll/pitchとgyro biasを推定し、orientationを持たないMid-360内蔵IMUでも初期化できるようにした。 |
| IMU単位変換 | 実機Livoxの `g` 入力とGazeboの `m/s^2` 入力をparameterで切り替え、preintegrationへ同じ単位で渡すようにした。 |
| IMU preintegration安定化 | 初期化中IMUの保持、fallback dt、ゼロ積分skip、GTSAM例外時のreset継続を追加した。 |
| scan matching初期値 | IMU preintegration odometryをscan matching初期値へ渡し、6軸IMUで壊れやすい並進成分は既定で使わないようにした。 |
| LiDAR処理queue化 | LiDAR callbackではenqueueだけを行い、timerで1scanずつ処理してrosbag再生時の取りこぼしを抑えるようにした。 |
| Livox CustomMsg補正 | 全点変換、空scan破棄、`offset_time` 診断、最大point timeによるscan終了時刻計算を追加した。 |
| deskew方式切替 | `imu_angular`、`odom_interpolation`、`off` を選択できるようにし、multi-LiDAR走行bagではsimulationでも時刻差をdeskew補正できるようにした。 |
| QoS強化 | LiDAR入力とCloudInfoのqueue depthを増やし、LIO-SAM内部pipelineのmessage dropを抑えるようにした。 |
| TF/frame整理 | LIO-SAMの動的TFを `odometryFrame -> lidarFrame` に統一し、base接続はproject側static TFへ分離した。 |
| override管理 | 固定commitのupstream checkoutへレビュー済みoverrideをコピーする方式にして、third_party改修を再現可能にした。 |

## 対象

| 項目 | 内容 |
|---|---|
| 対象リポジトリ | `https://github.com/UV-Lab/LIO-SAM_MID360_ROS2.git` |
| 固定commit | `066a3e44c6a8e3cb1bd1bdd49b2cb2365711a213` |
| override管理先 | `install/third_party_overrides/lio_sam_mid360_ros2/` |
| override適用処理 | `install/install_third_party.sh` |
| 実適用先 | `/opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws/src/lio_sam_mid360_ros2` |
| 既定のpackage名 | `lio_sam` |

## 管理方式

以前の文字列置換パッチスクリプト方式ではなく、現在はレビュー済みのoverrideファイルを固定commitのupstream checkoutへコピーする方式で管理している。

`install/install_third_party.sh` の `copy_lio_sam_mid360_overrides()` は、LIO-SAMのHEADが固定commitと一致することを確認してからoverrideをコピーする。commitが違う場合はコピーを拒否するため、upstream更新時に意図せず古いoverrideを上書きすることを防ぐ。

コピー対象は以下である。

| 種別 | ファイル |
|---|---|
| upstream同梱ファイル | `README.md` |
| upstream同梱ファイル | `LICENSE` |
| 改修対象 | `msg/CloudInfo.msg` |
| 改修対象 | `include/lio_sam/utility.hpp` |
| 改修対象 | `include/lio_sam/multi_lidar_projection_types.hpp` |
| 改修対象 | `include/lio_sam/multi_lidar_scan_synchronizer.hpp` |
| 改修対象 | `src/imageProjection.cpp` |
| 改修対象 | `src/featureExtraction.cpp` |
| 改修対象 | `src/mapOptmization.cpp` |
| 改修対象 | `src/imuPreintegration.cpp` |

## 改修全体像

| 分類 | 主な改修 | 主な対象 |
|---|---|---|
| 6軸IMU対応 | 静止区間の加速度からroll/pitch、gyro平均から初期biasを推定 | `utility.hpp`, `imageProjection.cpp`, `imuPreintegration.cpp` |
| IMU単位変換 | `g` と `m/s^2` をparameterで切替 | `utility.hpp`, launch/config |
| IMU preintegration安定化 | 初期化中IMU保持、fallback dt、ゼロ積分skip、GTSAM例外継続 | `imuPreintegration.cpp`, `utility.hpp` |
| scan matching初期値 | IMU preintegration odometryを初期値に使用し、並進/回転を個別制御 | `imageProjection.cpp`, `mapOptmization.cpp` |
| Livox CustomMsg処理 | 全点変換、空scan guard、`offset_time` 診断、scan終了時刻補正 | `imageProjection.cpp` |
| LiDAR queue化 | callbackはenqueueのみ、5ms timerで1scanずつ処理 | `imageProjection.cpp` |
| Multi-LiDAR同期 | 基準LiDAR近傍scanをマッチし、group開始時刻はマッチ済みscan群の最古stampへ揃える | `multi_lidar_scan_synchronizer.hpp`, `imageProjection.cpp` |
| deskew切替 | `imu_angular`, `odom_interpolation`, `off` を選択可能化 | `utility.hpp`, `imageProjection.cpp`, config |
| QoS強化 | LiDARとCloudInfoのqueue depthを増やし、CloudInfoをreliable化 | `utility.hpp`, `imageProjection.cpp`, `featureExtraction.cpp`, `mapOptmization.cpp` |
| TF/frame整理 | `odometryFrame -> lidarFrame` をLIO-SAM推定TFにし、base接続はproject側static TFに分離 | `mapOptmization.cpp`, `imuPreintegration.cpp`, launch/config |
| map保存拡張 | `/lio_sam/save_map` でlocalization PCDと2.5D global elevation mapを保存する | `mapOptmization.cpp`, `featureExtraction.cpp` |

## 追加パラメータ

| パラメータ | 意味 | LIO-SAM本体既定 | 実機YAML既定 | simulation YAML既定 |
|---|---|---|---|---|
| `imuType` | `six_axis` / `nine_axis` の切替 | `six_axis` | `six_axis` | `six_axis` |
| `imuAccelerationUnit` | 入力IMU加速度の単位 | `g` | `g` | `g` |
| `imuAccelerationScale` | 加速度へ追加で掛けるscale | `1.0` | `1.0` | `1.0` |
| `imuFrequency` | IMU時刻差が取れない場合のfallback周波数 | `500.0` | `500.0` | `200.0` |
| `imuDebug` | 変換後IMUのthrottleログ出力 | `false` | `false` | `false` |
| `waitForImuInitialization` | 6軸IMU初期姿勢推定完了までscan処理を待つ | `true` | `true` | `true` |
| `initialImuExpectedAccelerationNorm` | 初期静止判定で期待する加速度norm | `1.0` | `1.0` | `1.0` |
| `initialImuAccelerationNormTolerance` | 初期静止判定の加速度norm許容差 | `0.35` | `0.35` | `3.5` |
| `initialImuMaxAngularVelocity` | 初期静止判定の角速度上限 | `0.2` | `0.2` | `0.2` |
| `initialImuMinSamples` | 初期推定に必要な最小IMU sample数 | `50` | `50` | `50` |
| `initialImuMinDuration` | 初期推定に必要な最小時間 | `0.5` | `0.5` | `0.5` |
| `useImuPreintegrationInitialGuess` | IMU preintegration odometryをscan matching初期値へ使う | `true` | `true` | `true` |
| `useImuTranslationInitialGuess` | preintegration並進成分を初期値へ使う | `false` | `false` | `false` |
| `useImuRotationInitialGuess` | preintegration回転成分を初期値へ使う | `true` | `true` | `true` |
| `deskewMode` | scan内deskew方式 | `imu_angular` | `odom_interpolation` | `odom_interpolation` |
| `maxPointOffsetTimeSec` | Livox `offset_time` 異常診断の上限秒 | `0.2` | `0.2` | `0.2` |
| `hybridRegisteredCloudEnabled` | 旧hybrid local点群のpublish有効化。通常のmap保存では使わない | `false` | `false` | `false` |
| `hybridRegisteredCloudRawNearRange` | hybrid点群でraw deskew済み近傍点を優先するLiDAR距離 | `3.0` | `3.0` | `3.0` |
| `hybridRegisteredCloudRawNearLeafSize` | hybrid点群のraw詳細成分に適用するVoxelGrid leaf size。`0.0`で無効 | `0.01` | `0.01` | `0.01` |
| `hybridRegisteredCloudRawUpperMapZLimitEnabled` | 近傍raw詳細点群にmapFrame Z上限を適用するか | `true` | `true` | `true` |
| `hybridRegisteredCloudRawUpperMapZMax` | 近傍raw詳細点群から除外するmapFrame Z上限 | `1.5` | `1.5` | `1.5` |
| `hybridRegisteredCloudTopic` | 旧hybrid点群のpublish topic。通常は無効 | `/lio_sam/mapping/cloud_registered_legacy` | 未指定 | 未指定 |
| `imageProjectionMaxScansPerTimer` | backlog時に1 timer callback内で処理する最大scan数 | `5` | `5` | `5` |
| `imageProjectionBacklogLogThreshold` | backlog警告と早期処理時間ログを出すqueue長 | `10` | `10` | `10` |
| `processingTimeLogIntervalSec` | imageProjectionとhybrid点群生成の平均処理時間ログ周期。`0.0`で無効 | `5.0` | `5.0` | `5.0` |
| `publishDeskewedCloud` | `/lio_sam/deskew/cloud_deskewed` の外部publish | `false` | `false` | `false` |
| `publishFeatureClouds` | `/lio_sam/feature/cloud_corner` と `/lio_sam/feature/cloud_surface` の外部publish | `false` | `false` | `false` |
| `publishMapGlobalCloud` | `/lio_sam/mapping/map_global` のpublish | `false` | `false` | `false` |
| `publishMapLocalCloud` | `/lio_sam/mapping/map_local` のpublish | `false` | `false` | `false` |
| `publishTrajectoryCloud` | `/lio_sam/mapping/trajectory` のpublish | `false` | `false` | `false` |
| `publishCloudRegistered` | `/lio_sam/mapping/cloud_registered` のpublish | `false` | `false` | `false` |
| `saveElevationMap` | `/lio_sam/save_map` 保存時にlocalization/elevation用のdeskew済み全点群を内部保持する | `false` | `false` | `false` |
| `localizationSubmapLeafSize` | localization用keyframe raw submapの内部voxel leaf size | `0.10` | `0.10` | `0.10` |
| `saveLioSamStandardPcds` | LIO-SAM標準PCD群を `/lio_sam/save_map` で追加保存する | `false` | `false` | `false` |
| `elevationOutputCellSize` | 最終CSV出力用2.5D gridのセルサイズ | `0.01` | `0.01` | `0.01` |
| `elevationClusterCellSize` | z-cluster判定・global layer選択用gridのセルサイズ | `0.30` | `0.30` | `0.30` |
| `elevationCellZClusterGap` | セル内zクラスタの連続判定幅 | `0.05` | `0.05` | `0.05` |
| `elevationClusterConnectionRadius` | seed clusterから接続するglobal clusterのXY半径 | `0.45` | `0.45` | `0.45` |
| `elevationClusterConnectionZGap` | seed clusterから接続するglobal clusterの代表高さ差 | `0.15` | `0.15` | `0.15` |
| `elevationMinRange` | 2.5D抽出に使うLiDAR点の最小距離 | `0.0` | `0.0` | `0.0` |
| `elevationMaxRange` | 2.5D抽出に使うLiDAR点の最大距離 | `1000.0` | `1000.0` | `1000.0` |
| `publishCloudRegisteredRaw` | `/lio_sam/mapping/cloud_registered_raw` の外部publish。map保存には不要 | `false` | `false` | `false` |
| `publishLoopClosureClouds` | loop closure可視化用点群/markerのpublish | `false` | `false` | `false` |

`initialImuExpectedAccelerationNorm` と `initialImuAccelerationNormTolerance` は、`imuConverter()` で単位変換する前のraw IMU値に対して使う。simulationでは `mid360_sim_adapter` が実機相当の `g` 単位へ変換するため、live/bagとも実機側と同じ単位前提になる。

SLAM本体の性能・挙動値はCLIやlaunch引数ではなく、`lio_sam_mid360.yaml` と `lio_sam_mid360_sim.yaml` に記述する。LiDAR fusionの入力topic、基準LiDAR topic、fusion後CustomMsg topic、LIO-SAM入力topicは `multi_lidar_fusion.yaml` に記述し、基準LiDAR frameは基準topicの `CustomMsg.header.frame_id` から自動取得する。launch引数はRViz、rosbag再生などの実行制御に限定する。

## `utility.hpp`

### 6軸IMU初期姿勢推定

`imuType` を追加し、`six_axis` と `nine_axis` を切り替えられるようにした。

6軸IMUではorientation quaternionを絶対姿勢として信用できないため、起動直後の静止区間からroll/pitchだけを推定する。yawは6軸IMU単体では観測できないため0基準の相対値として扱う。

推定処理は以下の条件を満たすIMU sampleだけを使う。

| 条件 | 目的 |
|---|---|
| 加速度と角速度が有限値 | NaN/inf混入を避ける |
| 加速度normが期待値付近 | 重力のみを観測している静止状態に近いことを確認する |
| 角速度normが上限以下 | 回転中のsampleを初期姿勢に使わない |
| sample数と継続時間が閾値以上 | 瞬間的な揺れで初期姿勢が決まることを避ける |

条件を外れた場合はaccumulatorをresetし、静止sampleの収集をやり直す。必要sampleが揃うと、加速度平均からroll/pitch、gyro平均から初期gyro biasを計算して保持する。

### IMU単位変換

`imuAccelerationUnit` と `imuAccelerationScale` を追加し、入力加速度をpreintegrationが期待する `m/s^2` へ正規化する。

| 入力 | 変換 |
|---|---|
| `imuAccelerationUnit: g` | `imuAccelerationScale * imuGravity` を掛ける |
| `imuAccelerationUnit: mps2` | `imuAccelerationScale` のみ掛ける |

その後、既存の `extrinsicRot` を使ってIMU座標からLiDAR座標へ回転する。

### gyro bias補正

6軸IMUの初期化が完了している場合、静止区間から推定した `sixAxisInitialGyroBias` をraw角速度から差し引く。その後、`extrinsicRot` でLiDAR座標へ回転する。

この補正は、静止時の微小biasがpreintegrationの姿勢driftとして蓄積することを抑えるためのものである。

### orientation生成

`imuConverter()` のorientation処理はIMU種別で分岐する。

| IMU種別 | orientation処理 |
|---|---|
| `six_axis` | 初期推定roll/pitchとyaw=0からquaternionを作る |
| `nine_axis` | 入力orientationを使う |

どちらの場合も最後に `extQRPY` を掛け、LiDAR/IMU外部姿勢を反映してから正規化する。9軸IMUでは入力orientationを使うため、無効quaternionの検出は正規化後の限定的なguardに留まる。

### 共通helper

以下のhelperを追加して、LIO-SAM各nodeから同じ判断を使えるようにした。

| helper | 役割 |
|---|---|
| `usingSixAxisImu()` | 6軸IMUモードかを返す |
| `sixAxisImuReady()` | 6軸IMU初期姿勢が利用可能かを返す |
| `shouldWaitForSixAxisImuInitialization()` | scan処理やIMU odometry publishを待つべきかを返す |
| `fallbackImuDeltaTime()` | `imuFrequency` 由来のfallback dtを返す |
| `accelerationScaleToMps2()` | 加速度単位変換scaleを返す |

### QoS変更

LiDAR入力QoSのdepthを `200` に増やした。rosbag再生やsimulationで短時間にscanが集中した場合でも、callback処理の遅れで即座にdropしにくくするためである。

## `imageProjection.cpp`

### LiDAR callbackのqueue化

従来はLiDAR callback内で点群変換、deskew、range image投影、CloudInfo publishまで実行していた。これを、callbackでは `CustomMsg` をqueueへ積むだけに変更した。

重い処理は5ms周期のtimer `processCloudQueue()` が担当する。通常は到着したscanだけ処理し、backlog時は `imageProjectionMaxScansPerTimer` を上限に1 callback内で複数scanを連続処理する。

queue操作は以下の責務に分けている。

| 処理 | 責務 |
|---|---|
| `cloudHandler()` | LiDAR callbackで受けたscanをenqueueする |
| `processCloudQueue()` | queue先頭を処理し、成功または破棄時だけpopする |
| `cachePointCloud()` | queue先頭scanを変換し、変換不能ならfalseを返す |

点群変換中は `cloudLock` を保持しない。rosbag再生中やsimulationでLiDAR callbackが連続しても、重い変換処理がcallback enqueueを長時間塞がないようにしている。

`/lio_sam/deskew/is_cloud_queue_empty` serviceを追加した。既存の `lio_sam/srv/SaveMap` 型を流用し、`success=true` を「未処理scanなし」として返す。`scripts/run_slam.sh --bag-play` はbag再生終了後にこのserviceをpollし、`cloudQueue` が空になるまでSLAM停止を待つ。

`processingTimeLogIntervalSec` 周期で `ImageProjection timing` ログを出す。`cache`、`deskew`、`project`、`extraction`、`publish`、近傍raw voxel処理時間、近傍raw点数、queue長を平均値として出力し、処理律速箇所を追えるようにした。

`publishDeskewedCloud` は既定で無効にしている。`cloud_deskewed` はCloudInfo内部では引き続きfeatureExtractionへ渡すが、外部topicとしてはpublishしないため、全topic録画時に診断用点群が余分に流れない。

### `DeskewStatus`

`deskewInfo()` の戻り値を `bool` ではなく `DeskewStatus` にした。

| 状態 | 意味 | queue操作 |
|---|---|---|
| `Ready` | 必要情報が揃いscan処理可能 | 処理後にpop |
| `Wait` | IMUやodomがまだ到着していない | popせず次timerで再試行 |
| `Drop` | 必要時刻のIMUやodomが既に失われ、復旧不能 | popして次scanへ進む |

この分離により、単なる到着待ちのscanを誤って破棄することと、復旧不能な古いscanがqueue先頭で詰まり続けることの両方を避ける。

### Livox `CustomMsg` 変換

Livox `CustomMsg` からLIO-SAM内部点型へ変換する処理を修正した。

| 改修 | 理由 |
|---|---|
| `point_num` の全点を変換 | 末尾点を落とさずscan全体を使う |
| 空scanをguard | 空点群で時刻計算や投影に進まない |
| `offset_time` のmin/maxを監視 | 単位違いや異常なscan durationを検出する |
| `offset_time` の単調性を監視 | scan内時刻が逆転する入力を診断する |
| scan終了時刻を最大point timeから算出 | 最後の配列要素が最大時刻とは限らない入力に対応する |

`maxPointOffsetTimeSec` を超えるpoint timeが入った場合はthrottle付きwarningを出す。これはdeskew品質に直結するため、simulation adapterやrosbag変換の不備を検出する目的である。

### 6軸IMU初期化待ち

`waitForImuInitialization` が有効で、6軸IMUの初期roll/pitch推定が未完了の場合、scanを `Wait` にしてLiDAR queue先頭に保持する。

初期化前に受信したIMUは変換済みqueueへ入れず、raw IMU queueへ保持する。初期roll/pitchとgyro biasが確定した時点でraw IMU履歴を再変換し、その後に保持していたLiDAR scanを先頭から順番に処理する。

この設計により、初期姿勢未確定だけを理由にscanを捨てず、かつ初期姿勢確定前に作られた不正確な変換済みIMUでdeskewしないようにしている。

### deskew方式切替

`deskewMode` によりscan内deskew方式を切り替える。

| mode | 動作 | 主な用途 |
|---|---|---|
| `imu_angular` | IMU角速度をscan内で積分し、各点をscan開始姿勢へ戻す | 従来LIO-SAMに近い動作 |
| `odom_interpolation` | IMU preintegration odometryを各点時刻へ補間してdeskewする | 実機、bag再生、multi-LiDAR simulationの既定 |
| `off` | 点群を補正せずそのまま投影する | Gazebo理想入力や低速検証 |

`imu_angular` ではscan終了時刻までのIMUが必要になる。足りない場合は `Wait`、scan開始より前のIMUが既にない場合は `Drop` とする。

`odom_interpolation` ではIMU角速度積分への依存を下げ、IMU preintegration odometryをscan内時刻へ補間する。位置は線形補間、姿勢はslerpで補間する。

### IMU preintegration odometry補間

`odomTopic + "_incremental"` を購読し、IMU preintegrationの高頻度odometryをscan開始、scan終了、各点時刻へ補間する。

scan開始時刻の補間poseは、`CloudInfo.initial_guess_*` に格納してmapOptimizationへ渡す。これにより、mapOptimizationはscan時刻に近いIMU preintegration結果をscan matching初期値として使える。

起動直後は、まだIMU preintegration odometryが存在しないため、scan開始時刻へのodom補間が一度成立するまでodomなしでscan publishを許可する。これによりmapping側の補正が生成され、その後のIMU preintegration odometryが開始できる。

### CloudInfo QoS

`lio_sam/deskew/cloud_info` のpublish QoSを `reliable + KeepLast(200)` に変更した。

点群本体のsensor入力はbest effortのまま維持しつつ、LIO-SAM内部pipelineの制御情報であるCloudInfoは欠落しにくくする方針である。

### `/lio_sam/save_map` とmap保存

`mapOptmization.cpp` は、既存の `/lio_sam/save_map` serviceを拡張し、`cloudInfo.cloud_deskewed` 由来のkeyframe raw submapから `localization_map.pcd` を保存する。`destination` が絶対パスならそのまま使い、相対パスなら従来互換として `$HOME + destination` へ保存する。

`featureExtraction.cpp` は `publishCloudRegisteredRaw || saveElevationMap` の場合だけ `cloudInfo.cloud_deskewed` を後段へ残す。通常運用では従来どおり空にしてCloudInfo通信量を抑える。

`mapOptmization.cpp` は各scan処理後に `cloudInfo.cloud_deskewed` を直近または新規keyframe submapへ集約し、raw点群そのものは保持しない。localizationはkeyframe local XYZ voxel、elevationはgravity-aligned keyframe XY/relative Z cellごとの複数z-cluster統計として保持する。保存時はloop closure補正後keyframe poseで各clusterをglobalへ剛体再配置し、global 30cm cell内で同じ高さ帯のclusterを再結合する。その後、3D原点最近傍clusterをseedにして、XY半径0.45m以内かつ代表高さ差0.15m以内で接続するclusterだけを辿り、各30cm cellではseed高さに最も近いclusterを1つ採用する。最終CSVには、採用clusterと対応する1cm output cellの統計だけを出力する。

localization用PCDは、submap内部で `localizationSubmapLeafSize` により0.10m voxel統計へ集約し、保存時に `SaveMap.request.resolution` でglobal voxel downsampleしたraw downsample mapを `localization_map.pcd` としてbinary PCDで保存する。

2.5D mapは単一の `global_elevation_map.csv` と `elevation_manifest.yaml` として保存する。CSVの `count,z_min,z_max,z_mean,z_m2,height_range` は、原点最近傍seedから接続するglobal clusterのうち、各30cm cellで採用されたcluster由来である。CSV列は `ix,iy,x,y,count,z_min,z_max,z_mean,z_m2,lowest_cluster_count,lowest_cluster_min,lowest_cluster_max,lowest_cluster_mean,height_range` である。

`saveLioSamStandardPcds=false` の既定では、`GlobalMap.pcd` / `CornerMap.pcd` / `SurfMap.pcd` / `trajectory.pcd` / `transformations.pcd` は生成しない。デバッグでfeature mapが必要な場合だけ `saveLioSamStandardPcds=true` にする。

保存用scriptは旧Trigger serviceではなく `/lio_sam/save_map lio_sam/srv/SaveMap` を呼ぶ。service responseはsuccessのみのため、出力path表示はscript側の既知の保存先から補完する。

旧hybrid点群生成と `cloud_registered_raw` 外部publishは通常のmap保存では使わず、診断用途の明示有効化だけに残す。

## `featureExtraction.cpp`

`imageProjection -> featureExtraction -> mapOptimization` 間のCloudInfoを落としにくくするため、CloudInfoのsubscribe/publish QoSを `reliable + KeepLast(200)` に変更した。

特徴抽出アルゴリズム自体は大きく変更していない。主な目的は、rosbag再生や高頻度入力時にCloudInfoだけが欠落し、後段のmapOptimizationがscanを受け取れなくなる状態を避けることである。

`publishFeatureCloud()` では全点deskew済み点群 `cloud_deskewed` だけを空にしてからpublishする。近傍raw詳細点群 `cloud_deskewed_raw_near` は保持し、corner/surface feature点群と同じCloudInfoでmapOptimizationへ渡す。

## `mapOptmization.cpp`

### CloudInfo QoS

`lio_sam/feature/cloud_info` のsubscribe QoSを `reliable + KeepLast(200)` に変更した。

featureExtraction側と同じQoSに揃えることで、LIO-SAM内部pipelineでCloudInfoが落ちる可能性を下げる。

### IMU preintegration初期値

`updateInitialGuess()` を拡張し、`CloudInfo.initial_guess_*` に格納されたIMU preintegration odometryをscan matching初期値へ使えるようにした。

初期値として使うのは、前回scan開始時刻のpreintegration poseから今回scan開始時刻のpreintegration poseまでの相対変化である。これを現在の `transformTobeMapped` に掛け、scan matching開始前の予測poseにする。

6軸IMUでは重力方向のroll/pitchは得られる一方、yawや並進はdriftしやすい。そのため、以下のparameterで並進と回転を個別に採用できる。

| パラメータ | 既定 | 意図 |
|---|---|---|
| `useImuPreintegrationInitialGuess` | `true` | preintegration初期値を使うか |
| `useImuTranslationInitialGuess` | `false` | 並進driftをscan matching初期値へ入れない |
| `useImuRotationInitialGuess` | `true` | 回転変化だけは初期値へ使う |

preintegration初期値を使わない場合でも、従来のIMU roll/pitch/yaw初期値から得た相対回転はfallbackとして使う。

### roll/pitch融合の共通化

`blendRollPitchWithImu()` を追加し、`transformUpdate()` とincremental odometry publishの両方で同じroll/pitch融合を使うようにした。

これにより、global odometryとincremental odometryでIMU roll/pitch補正の重みがずれることを避ける。重みは既存parameterの `imuRPYWeight` を使う。

現在の `lio_sam_mid360.yaml` では `imuRPYWeight: 0.0` としている。これは6軸IMUの初期roll/pitchは初期化や初期値には使うが、scan matching後の姿勢へ継続的には混ぜない安全側設定である。

### odometry/TF frame整理

公開odometryとTFを `odometryFrame -> lidarFrame` に統一した。

| 出力 | frame |
|---|---|
| `lio_sam/mapping/odometry` | `header.frame_id = odometryFrame`, `child_frame_id = lidarFrame` |
| `lio_sam/mapping/odometry_incremental` | `header.frame_id = odometryFrame`, `child_frame_id = lidarFrame` |
| TF | `odometryFrame -> lidarFrame` |
| path | `odometryFrame` |
| key pose cloud | `odometryFrame` |
| registered cloud | `odometryFrame` |

本プロジェクトでは `odometryFrame = lidar_init`、`lidarFrame = lidar_odom` としている。LIO-SAMが推定する動的frameをURDF上の実LiDAR frameから分離し、base接続はlaunch側のstatic TFで行う。

## `imuPreintegration.cpp`

### mapping odometry購読topic

IMU preintegrationの補正入力は `lio_sam/mapping/odometry_incremental` を購読する。

mapOptimizationがpublishしたincremental odometryを使って、IMU preintegrationのfactor graphを定期補正する。`odometry_incremental_internal` のような内部専用topicは使わず、topicを単純化している。

### 初期化中IMUの保持

6軸IMUの初期roll/pitch推定が完了していない間も、IMU sampleを `imuQueOpt` と `imuQueImu` へ積むようにした。

初期化完了前にmapping補正が先に到着した場合でも、補正時刻以前のIMUがqueueに残っていれば初回preintegrationを開始できる。これにより、初回mapping補正を捨て続けてIMU odometryが立ち上がらない状態を避ける。

### fallback dt

IMU時刻差がまだ計算できない初回sampleでは、固定値ではなく `fallbackImuDeltaTime()` を使う。これは `1.0 / imuFrequency` で計算される。

実機Mid-360は500Hz、Gazebo simulationは200Hzの既定値を使うため、入力に合ったdtでpreintegrationを開始できる。

### ゼロ積分時間のskip

LiDAR補正時刻までに積分できたIMU durationがほぼ0の場合、GTSAM graphへIMU factorを追加せず補正をskipする。

IMU factorに有効な時間幅がない状態でoptimizerへ渡すと、数値的に不安定になりやすい。warningを出して次の補正を待つことで、processを落とさず継続する。

### GTSAM optimizer例外処理

初回optimizer updateと通常optimizer updateの両方で例外を捕捉するようにした。

例外が発生した場合はwarningを出し、graphや状態をresetして次の補正を待つ。rosbagやsimulationで一時的に不整合な時刻や姿勢が入っても、node全体が終了しないようにするためである。

### failureDetectionログ強化

速度異常とbias異常のログにnormと各成分を出すようにした。

| 異常 | 追加情報 |
|---|---|
| velocity | norm, x, y, z |
| accelerometer bias | norm, x, y, z |
| gyroscope bias | norm, x, y, z |

実機調整時に、単にresetした事実だけでなく、どの成分がどの程度破綻したかを確認できる。

### TransformFusionのTF抑制

`TransformFusion` はIMU odometryとLiDAR odometryの融合結果をpublishするが、base系TFはproject側static TFへ集約した。

そのため、`TransformFusion` から `base_footprint` などへのTFを送らないようにしている。TF tree上でLIO-SAM由来の動的base TFとURDF/static TFが競合することを避けるためである。

### IMU odometry queue安全化

LiDAR補正時刻より古いIMU odometryを捨てる処理で、queueに1要素しか残らない状態でもfront/back参照が壊れないようにした。

補正時刻と同時刻のsampleだけが残るケースはrosbag再生や低速simulationで起こりやすいため、防御的に `size() > 1` を条件にしている。

## 実機向けlaunch/config

主な実機向け設定は `ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml` と `ros2_ws/src/ai_ship_robot_slam/launch/lio_sam.launch.py` で管理している。

### 実機既定値

| 項目 | 既定値 |
|---|---|
| `input_custom_topics` | `/lidar1/livox/lidar`, `/lidar2/livox/lidar` |
| `input_ring_offsets` | `0`, `4` |
| `imuTopic` | `/lidar1/livox/imu` |
| `sensor` | `livox` |
| `N_SCAN` | `8` |
| `Horizon_SCAN` | `10000` |
| `imuAccelerationUnit` | `g` |
| `imuFrequency` | `500.0` |
| `deskewMode` | `odom_interpolation` |
| `useImuTranslationInitialGuess` | `false` |
| `useImuRotationInitialGuess` | `true` |
| `mappingProcessInterval` | `0.0` |
| `loopClosureEnableFlag` | `false` |

### TF構成

LIO-SAM本体には、以下のframeを渡す。

| LIO-SAM parameter | 本プロジェクトでの値 | 役割 |
|---|---|---|
| `mapFrame` | `map` | 全体map frame |
| `odometryFrame` | `lidar_init` | LIO-SAM推定の親frame |
| `lidarFrame` | `lidar_odom` | LIO-SAM推定の子frame |
| `baselinkFrame` | `base_footprint` | ロボット基準frame |

`lio_sam.launch.py` は `multi_lidar_fusion.yaml` の `reference_custom_topic` で指定した基準LiDAR CustomMsgから `header.frame_id` を取得し、`/tf_static` の `base_footprint -> <基準LiDAR frame>` から以下のstatic TFをpublishする。

| static TF | 目的 |
|---|---|
| `map -> lidar_init` | URDF上の基準LiDAR配置から、SLAM初期frameの位置とyawを与える |
| `lidar_odom -> base_footprint` | LIO-SAM推定LiDAR poseをロボットbaseへ接続する |

LIO-SAMの動的TFは `lidar_init -> lidar_odom` に限定する。これにより、URDFの実LiDAR linkやbase linkとの責務が分離される。

### 任意node

`lio_sam.launch.py` には、LIO-SAM本体以外に以下の補助nodeがある。

| node | 既定 | 目的 |
|---|---|---|
| `slam_reference_lidar_static_tf_node` | 有効 | 基準LiDAR CustomMsgと`/tf_static`からSLAM用static TFを生成 |

## simulation向けlaunch

`sim/ros2_ws/src/ai_ship_robot_gazebo/launch/sim_lio_sam.launch.py` は、本番用 `lio_sam.launch.py` をincludeし、simulation固有の差分だけをlaunch argumentで上書きする。

### simulation既定値

| 項目 | 既定値 |
|---|---|
| `use_sim_time` | `true` |
| Gazebo LiDAR raw topic | `/lidar1/custom` |
| Gazebo IMU raw topic | `/lidar1/imu` |
| LIO-SAM LiDAR topic | `/lidar1/livox/lidar`, `/lidar2/livox/lidar` |
| LIO-SAM IMU topic | `/lidar1/livox/imu` |
| `use_mid360_sim_adapter` | `true` |
| `imuAccelerationUnit` | `g` |
| `imuFrequency` | `200.0` |
| `initialImuExpectedAccelerationNorm` | `1.0` |
| `initialImuAccelerationNormTolerance` | `3.5` |
| `deskewMode` | `odom_interpolation` |
| Gazebo odom TF | `publish_odom_tf=false` |

Gazebo raw IMUはROS標準の `m/s^2` 単位で出るが、SLAMでは `mid360_sim_adapter` 後の `/lidar1/livox/imu` を使うため、実機Livox相当の `g` 入力として扱う。

simulationでも左右LiDARのscan stampに差があるbagではyaw回転中の点群ずれがPCD mapへ残るため、既定の `deskewMode` は `odom_interpolation` にしている。各点の `offset_time` は0固定でも、LiDAR間のscan stamp差はgroup開始基準の相対時刻として補正する。

## topic関係

主要topicの流れは以下である。

| producer | topic | consumer | 用途 |
|---|---|---|---|
| Livox driver / sim adapter | `/lidar1/livox/lidar` | `lio_sam_imageProjection`, `slam_reference_lidar_static_tf_node` | 基準LiDARのLivox `CustomMsg` scan |
| Livox driver / sim adapter | `/lidar2/livox/lidar` | `lio_sam_imageProjection` | 追加LiDARのLivox `CustomMsg` scan |
| Livox driver / Gazebo | `/lidar1/livox/imu` または `/lidar1/imu` | `lio_sam_imageProjection`, `lio_sam_imuPreintegration` | IMU入力 |
| `lio_sam_imageProjection` | `lio_sam/deskew/cloud_info` | `lio_sam_featureExtraction` | deskew済み点群と初期値情報 |
| `lio_sam_featureExtraction` | `lio_sam/feature/cloud_info` | `lio_sam_mapOptimization` | 特徴点抽出後のscan情報 |
| `lio_sam_mapOptimization` | `lio_sam/mapping/odometry_incremental` | `lio_sam_imuPreintegration` | LiDAR補正によるIMU preintegration補正入力 |
| `lio_sam_imuPreintegration` | `odometry/imu_incremental` | `lio_sam_imageProjection`, `lio_sam_transformFusion` | 高頻度IMU preintegration odometry |
| `lio_sam_transformFusion` | `odometry/imu` | 外部可視化/利用先 | LiDAR補正を反映したIMU odometry |
| `lio_sam_mapOptimization` | `lio_sam/mapping/odometry` | 外部可視化/利用先 | scan matching後のglobal odometry |
| `lio_sam_mapOptimization` | `lio_sam/mapping/cloud_registered` | RViz / 外部利用先 | local frameのdownsample済みfeature点群 |
| `lio_sam_mapOptimization` | `/lio_sam/save_map` | scripts / operator | 既存PCD、`localization_map.pcd`、2.5D elevation mapの保存service |

`odometry/imu_incremental` は `odomTopic + "_incremental"` で決まる。現在のYAMLでは `odomTopic: odometry/imu` である。

## 運用上の注意

LIO-SAM本体の改修正本は `install/third_party_overrides/lio_sam_mid360_ros2/` のoverrideファイルである。`/opt/ai_ship_robot/.../third_party_ws/src/lio_sam_mid360_ros2` 側のファイルは、install時にコピーされた生成物として扱う。

upstream commitを変更する場合は、`install/install_third_party.sh` の固定commit確認でoverrideコピーが拒否される。新しいupstreamに対して管理対象のoverrideファイルを再移植し、ビルドと実行確認をやり直す必要がある。

6軸IMU初期化には静止区間が必要である。起動直後にロボットが動いているとaccumulatorがresetされ続け、scanがdropされる。

`useImuTranslationInitialGuess` は既定で `false` にしている。6軸IMUと低速台車ではpreintegration並進がdriftしやすく、scan matching初期値へ入れると悪化する可能性がある。

simulation bagを含むmulti-LiDAR走行検証では、LiDAR間のscan stamp差を補正するため `deskewMode: odom_interpolation` を使う。IMU/odom時刻待ちを避けたい単体の初期切り分け時のみ、必要に応じて `off` へ一時変更する。

TF構成は `lio_sam.launch.py` のstatic TFとセットで成立する。LIO-SAM nodeだけを直接起動する場合、`map -> lidar_init` と `lidar_odom -> base_footprint` の接続が不足する可能性がある。

CloudInfoはreliable QoSにしているため、subscriberが処理できないほど詰まると遅延が増える可能性がある。rosbag記録や再生で追いつかない場合は、再生速度や記録topicを絞る方が根本対策になる。

性能検証や通常のSLAM出力記録では `--bag-topics /lio_sam/mapping/cloud_registered_raw,/lio_sam/mapping/odometry,/lio_sam/mapping/path,/clock,/tf_static` のように必要topicを明示し、不要な可視化topicを含めない。

## 確認コマンド

third_partyの取得、override適用、ビルドは以下で行う。

```bash
bash install/install_third_party.sh
```

実機向けLIO-SAM launchの主な起動対象は以下である。

```bash
ros2 launch ai_ship_robot_slam lio_sam.launch.py
```

simulation込みのLIO-SAM launchの主な起動対象は以下である。

```bash
ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py
```

実際の起動はプロジェクトのrun scriptから行う場合があるため、運用時は `scripts/run_slam.sh` や `sim/scripts/run_simulation.sh` の引数も確認する。
