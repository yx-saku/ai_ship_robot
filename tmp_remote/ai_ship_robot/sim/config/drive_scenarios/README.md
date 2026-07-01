# Drive Scenario Reference

このディレクトリには、`sim/scripts/drive_robot.sh --scenario` と `sim/scripts/run_simulation.sh --drive-scenario` で使う YAML シナリオを配置します。

## 基本形式

```yaml
vars:
  move_x: 1.0

steps:
  - duration_sec: 1.0
    commands:
      - type: stop

  - duration_sec: 5.0
    move_to_pose:
      type: abs
      pos:
        x: ${move_x}
        y: 1.0
        tolerance: 0.05
      yaw:
        deg: 90.0
        tolerance: 0.5
```

## トップレベル

- `vars`: 任意。変数名から数値または式文字列へのマップ
- `steps`: 必須。1 件以上の step entry を持つ配列

## vars

- 変数名は英数字と `_` を使い、先頭は英字または `_`
- 値は数値、または `${...}` で囲んだ式文字列
- 参照可能な演算子は `+` `-` `*` `/` と括弧 `(` `)`
- 変数は定義順に評価され、後ろの変数から前の変数は参照できません

## 通常 step

通常 step は `duration_sec` と、`commands` または `move_to_pose` のどちらか一方を持ちます。`duration_sec` はシミュレーション時間基準の秒数です。

### commands

```yaml
- duration_sec: 2.0
  commands:
    - type: forward
      speed: 0.2
```

| type | 動作 | speed 単位 |
| --- | --- | --- |
| `stop` | 停止 | なし |
| `forward` | `base_footprint` +X 方向へ移動 | m/s |
| `backward` | `base_footprint` -X 方向へ移動 | m/s |
| `left` | `base_footprint` +Y 方向へ移動 | m/s |
| `right` | `base_footprint` -Y 方向へ移動 | m/s |
| `yaw_left` | +Z 軸回りへ回転 | rad/s |
| `yaw_right` | -Z 軸回りへ回転 | rad/s |

### move_to_pose

```yaml
- duration_sec: 5.0
  move_to_pose:
    type: rel
    pos:
      x: 1.0
      y: 0.0
      tolerance: 0.05
    yaw:
      deg: 90.0
      tolerance: 0.5
```

- `type`: `abs` または `rel`。省略時は `abs`
- `abs`: `/odom` 原点基準の絶対位置・絶対 yaw へ移動
- `rel`: step 開始時のロボット位置・yaw を基準にした相対位置・相対 yaw へ移動
- `pos.x`, `pos.y`: m 単位
- `pos.tolerance`: m 単位。省略時は `0.05`
- `yaw.deg`: degree 単位
- `yaw.tolerance`: degree 単位。省略時は `1.0`
- `pos` を省略した場合は step 開始時の現在位置を維持します
- `yaw` を省略した場合は step 開始時の現在 yaw を維持します
- `pos` と `yaw` の両方を省略することはできません
- `move_to_pose` の `duration_sec` は速度算出用の目標到達時間であり、超過しても timeout にはしません

`pos` と `yaw` は省略記法でも指定できます。

```yaml
pos: [1.0, 1.0, 0.01]  # x=1.0, y=1.0, tolerance=0.01
pos: [1.0, 1.0]        # x=1.0, y=1.0, tolerance=0.05
yaw: [90.0, 0.5]       # deg=90.0, tolerance=0.5
yaw: [90.0]            # deg=90.0, tolerance=1.0
yaw: 90.0              # deg=90.0, tolerance=1.0
```

`pos` の flow sequence 内で `${...}` を使う場合は、`pos: ["${target_x}", "${target_y}"]` のようにクォートします。

`commands` と `move_to_pose` は同じ step では併用できません。

## repeat ブロック

```yaml
- repeat:
    count: 3
    steps:
      - duration_sec: 1.0
        commands:
          - type: forward
            speed: 0.2
      - duration_sec: 0.5
        commands:
          - type: stop
```

- `repeat.count`: 必須。1 以上の整数、または整数値に解決される `${...}`
- `repeat.steps`: 必須。1 件以上の step entry を持つ配列

## set ステップ

```yaml
- set:
    stride: ${stride + 1.0}
```

`set` は変数の更新だけを行い、走行コマンドは発行しません。
