# Drive Scenario Reference

このディレクトリには、`sim/scripts/drive_robot.sh --scenario` と `sim/scripts/run_simulation.sh --drive-scenario` で使う YAML シナリオを配置します。

## 形式

```yaml
vars:
  forward_speed: 0.2
  turn_speed: ${forward_speed * 2}
steps:
  - duration_sec: 3.0
    commands:
      - type: forward
        speed: ${forward_speed}
      - type: yaw_left
        speed: ${turn_speed}
```

## トップレベル

- `vars`: 任意。変数名から数値または式文字列へのマップ
- `steps`: 必須。1 件以上の step を持つ配列

## vars

- 変数名は英数字と `_` を使い、先頭は英字または `_`
- 値は数値、または `${...}` で囲んだ式文字列
- 参照可能な演算子は `+` `-` `*` `/` と括弧 `(` `)`
- 変数は定義順に評価され、後ろの変数から前の変数は参照できません

```yaml
vars:
  base_speed: 0.25
  turn_speed: ${base_speed / 2}
  short_wait: ${1 + 0.5}
```

## step entry

`steps` の各要素には、通常 step、`repeat` ブロック、`set` ステップを置けます。

### 通常 step

- `duration_sec`: 必須。0 より大きい秒数
- `commands`: 必須。1 件以上の command を持つ配列
- `duration_sec` は数値または `${...}` を指定できます

### set ステップ

```yaml
- set:
    stride: ${stride + 1.0}
    turn_speed: ${turn_speed * 0.8}
```

- `set` は変数の更新だけを行い、走行コマンドは発行しません
- 更新後の値は、後続の step / repeat から参照されます

### repeat ブロック

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

- `repeat.count`: 必須。1 以上の整数
- `repeat.count` は整数値に解決される `${...}` も指定できます
- `repeat.steps`: 必須。1 件以上の step entry を持つ配列
- `repeat.steps` の中でも通常 step、`repeat`、`set` をネストできます

## command

```yaml
- type: <command_type>
  speed: <non_negative_number>
```

- `type`: 必須
- `speed`: `stop` 以外で必須。0 以上の数値または `${...}`

## command type

| type | 動作 | 単位 |
| --- | --- | --- |
| `stop` | 停止 | なし |
| `forward` | `linear.x` 正方向 | m/s |
| `backward` | `linear.x` 負方向 | m/s |
| `left` | `linear.y` 正方向 | m/s |
| `right` | `linear.y` 負方向 | m/s |
| `yaw_left` | `angular.z` 正方向 | rad/s |
| `yaw_right` | `angular.z` 負方向 | rad/s |

## 制約

- `stop` は同一 step 内で単独指定します
- `forward` と `backward` は同時指定できません
- `left` と `right` は同時指定できません
- `yaw_left` と `yaw_right` は同時指定できません
- 異なる軸の command は同一 step 内で組み合わせできます

## 例

```yaml
vars:
  move_speed: 0.2
  turn_speed: ${move_speed * 2}
  wait_short: 0.5
  cycle_count: ${1 + 2}

steps:
  - duration_sec: 2.0
    commands:
      - type: stop

  - duration_sec: 3.0
    commands:
      - type: forward
        speed: ${move_speed}
      - type: yaw_right
        speed: ${turn_speed}

  - repeat:
      count: ${cycle_count}
      steps:
        - duration_sec: ${1.0 + wait_short}
          commands:
            - type: left
              speed: ${move_speed / 2}
        - duration_sec: ${wait_short}
          commands:
            - type: stop

  - set:
      move_speed: ${move_speed + 0.05}
```

現在このディレクトリには `around_world.yaml` と `yaw_right.yaml` があります。
