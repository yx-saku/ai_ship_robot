# Drive Scenario Reference

このディレクトリには、`sim/scripts/drive_robot.sh --scenario` または `sim/scripts/run_simulation.sh --drive-scenario` で使う走行シナリオYAMLを配置します。

## 基本構造

```yaml
steps:
  - duration_sec: 3.0
    commands:
      - type: forward
        speed: 0.2
      - type: yaw_left
        speed: 0.4
```

## Top Level

- `steps`: 必須。1つ以上のstepを持つリストです。

## Step Fields

- `duration_sec`: 必須。stepを継続するsim time秒数です。0より大きい数値を指定します。
- `commands`: 必須。1つ以上のcommandを持つリストです。

## Command Format

```yaml
- type: <command_type>
  speed: <non_negative_number>
```

`stop` 以外のcommandでは `speed` が必須です。`speed` は0以上の数値を指定します。

## Command Types

| type | 動作 | speed単位 |
| --- | --- | --- |
| `stop` | 停止する。`speed` は不要 | なし |
| `forward` | `base_footprint` +X方向へ移動する | m/s |
| `backward` | `base_footprint` -X方向へ移動する | m/s |
| `left` | `base_footprint` +Y方向へ横移動する | m/s |
| `right` | `base_footprint` -Y方向へ横移動する | m/s |
| `yaw_left` | +Z軸回りへ左回転する | rad/s |
| `yaw_right` | -Z軸回りへ右回転する | rad/s |

## Combination Rules

- `stop` は同一step内で単独指定します。
- `forward` と `backward` は同じ `linear_x` 軸なので同一step内で併用できません。
- `left` と `right` は同じ `linear_y` 軸なので同一step内で併用できません。
- `yaw_left` と `yaw_right` は同じ `angular_z` 軸なので同一step内で併用できません。
- 異なる軸のcommandは同一step内で組み合わせられます。

## Example

```yaml
steps:
  - duration_sec: 2.0
    commands:
      - type: stop

  - duration_sec: 3.0
    commands:
      - type: forward
        speed: 0.2
      - type: yaw_left
        speed: 0.4

  - duration_sec: 2.0
    commands:
      - type: right
        speed: 0.1
```
