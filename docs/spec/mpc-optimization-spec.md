# 周回走行解析機能（Telemetry Subsystem）設計仕様書 (Design Specification)

> **対象パッケージ**: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`  
> **最終更新**: 2026-08-15  
> **対象読者**: 人間開発者 / AIエージェント  
> **目的**: 自動走行データ解析環境、周回タイム集計、SW部門競技ルール適合判定（加速度制限）、走行軌跡・速度・加速度プロファイルの可視化基盤の仕様記録。

---

## 1. 全体仕様・目的概要

本設計仕様書は、Autoware Racing Kart の走行シミュレーションにおいて、走行テレメトリデータをミリ秒単位で自動収集・解析し、周回タイム計測、制御プロファイル可視化、および競技ルール適合性検証を行う解析サブシステムの構成を定義します。

すべての変更は提出制限ルールに従い、`aichallenge/workspace/src/aichallenge_submit/` ディレクトリ配下にのみ配置されています。

---

## 2. 競技ルール（SW部門）適合検証仕様

解析システムは、シミュレーション走行中に以下の競技ルール適合性を監視・検証します。

| 競技ルール項目 | 制約内容 | 解析システムでの検証・判定仕様 |
|---|---|---|
| **① 加速度上限** | 最大 **1.0 m/s²** | `/control/command/control_cmd` の目標加速度 $a_x(t)$ を常時監視。1.0 m/s² を超過したフレームの有無を検出し、レポートに PASS/FAIL を出力。 |
| **② 提出範囲制約** | `aichallenge_submit/` 配下のみ変更可 | 解析ノード・スクリプト・Launch設定をすべて `aichallenge_submit/` 内に完結。 |

---

## 3. 自動データ解析サブシステム仕様 (Telemetry Subsystem)

`make dev` の実行（シミュレーション起動・制御開始）と完全に同期して起動し、走行データを集計・プロット出力するシステムです。

### 3.1 構成ファイル一覧
1. **[`analyze_trajectory.py`](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/analyze_trajectory.py)**
   - ROS 2 トピック (`/localization/kinematic_state`, `/control/command/control_cmd`, `/mpc/ref_path`) を計測。
   - 周回検出（スタートライン通過判定）、ラップタイム集計、最大加速度のルール検証を実施。
   - `/output/YYYYMMDD-HHMMSS/trajectory_analysis.png` にプロット画像を出力。
2. **[`run_analyze_trajectory.bash`](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/run_analyze_trajectory.bash)**
   - ソースコードディレクトリ直参照フォールバック付きシェルラッパー（未ビルド時でもスクリプトを直接実行して Launch クラッシュを回避）。
3. **[`mpc.launch.xml`](../../aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/mpc.launch.xml)**
   - MPC起動時に解析ノードを自動同期起動。

```xml
<node pkg="multi_purpose_mpc_ros" exec="run_analyze_trajectory.bash"
      name="analyze_trajectory" output="screen">
  <param name="use_sim_time" value="$(var use_sim_time)"/>
</node>
```

### 3.2 購読トピック仕様

| トピック名 | 型 | 用途 |
|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | 実走行位置 $(x, y)$、実車速 $v$ の計測 |
| `/control/command/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | 目標加速度 $a_x$、操舵角 $\delta$ の計測 |
| `/mpc/ref_path` | `autoware_auto_planning_msgs/msg/Trajectory` | 参照走行ラインの取得・比較 |

### 3.3 解析出力画像 (`trajectory_analysis.png`) 構成
- **Subplot 1 (2D Driving Line)**: 参照ラインと車速カラーマップ付き実走軌跡プロット。
- **Subplot 2 (Velocity Profile)**: 距離 $s$ に対する車速推移 $v(s)$。
- **Subplot 3 (Acceleration Check)**: 加速度指令 $a_x(t)$ と 1.0 m/s² ルール超過検証グラフ。
- **Subplot 4 (Summary & Lap Times)**: 各周ラップタイム、ベストタイム、平均タイム、加速度ルール適合判定 (PASS/FAIL)。

---

## 4. 開発・ビルド・運用手順

```bash
# 1. 実行権限の付与 (初回)
chmod +x aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/*.bash
chmod +x aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/*.py

# 2. パッケージのビルド
make autoware-build

# 3. シミュレーション起動 (自動同期で解析ノードもスタート)
make dev
```
