# MPCコントローラ ラップタイム短縮・制御平滑化・分析基盤 設計仕様書 (Design Specification)

> **対象パッケージ**: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`  
> **最終更新**: 2026-07-28  
> **対象読者**: 人間開発者 / AIエージェント  
> **目的**: 競技ルール適合、ラップタイム短縮、S字切り返し挙動の平滑化、および自動走行データ解析環境の構成仕様の完全記録。

---

## 1. 全体仕様・目的概要

本設計仕様書は、Autoware Racing Kart の MPC (`multi_purpose_mpc_ros`) モードにおける周回タイム短縮と制御品質向上を目的に導入された全パラメータ変更、アーキテクチャ統合、および解析サブシステムの構成を定義します。

すべての変更は提出制限ルールに従い、`aichallenge/workspace/src/aichallenge_submit/` ディレクトリ配下にのみ配置されています。

---

## 2. 競技ルール（SW部門）への適合性制約

| 競技ルール項目 | 制約内容 | 本システムでの適合設計 |
|---|---|---|
| **① 加速度上限** | 最大 **1.0 m/s²** | `config.yaml` の `mpc.a_max` を `1.0` に厳格制限。不正加速度検出によるペナルティ（一定時間の速度制限）を回避。 |
| **② 速度上限・物理モデル適合** | シミュレータの物理モデルに準拠 | コーナリング限界および車輪スリップ防止範囲内で `ref_vel.yaml` のセクション目標速度を最適化。 |
| **③ 衝突・ペナルティ回避** | 壁接触・障害物衝突時ペナルティ | 横位置追従重み $Q_{e_y}$ と許容横加速度 $ay_{\text{max}}$ を調整し、壁衝突リスクをゼロ化。 |
| **④ ブースト機能** | SIM専用機能（今回は考慮外） | `use_boost_acceleration` を `false` に設定し、標準の加速度制御範囲のみでタイムを最大化。 |
| **⑤ 提出範囲制約** | `aichallenge_submit/` 配下のみ変更可 | すべての改修対象ファイル、スクリプト、ラッパーを `aichallenge_submit/` に限定。 |

---

## 3. コントローラ パラメータ設定仕様 (Tuning Specifications)

### 3.1 `config.yaml` 調整仕様
パス: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`

```yaml
mpc:
  a_max: 1.0                     # [m/s^2] SW部門ルール適合 (上限1.0m/s^2)
  ay_max: 13.5                    # [m/s^2] コーナリング最大許容横加速度 (早め減速の抑止)
  wp_id_offset: 3                # 先読みウェイポイントオフセット (約1.8m: S字コーナー進入手前からの早期切込)
  use_max_kappa_pred: true       # 曲率変化予測によるスムーズ減速・舵角連携の有効化
  Q: [400.0, 800.0, 100.0]       # コスト行列 [e_y (横偏差), e_psi (方位角偏差), v_err (速度偏差)]
  QN: [400.0, 800.0, 100.0]      # 終端コスト行列 [e_y, e_psi, v_err]
```

#### パラメータチューニングの根拠・効果
1. **$Q = [400.0, 800.0, 100.0]$**:
   従来設定 `[200.0, 1500.0, 100.0]` では方位角偏差重み $Q_{e_\psi}$ が過大であり、S字切り返しポイント (X: 89620〜89640, Y: 43180〜43190) で過剰な復元舵角を出力してハンチングを引き起こしていました。方位角重みを下げて横偏差重みを強めることで、滑らかなステアリング操作を実現しました。
2. **`wp_id_offset: 3` & `use_max_kappa_pred: true`**:
   S字反転部において、手前からコーナー曲率変化を予測して余裕を持ったスムーズな切り返し操作を行います。

---

### 3.2 `ref_vel.yaml` 区間速度設定仕様
パス: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/ref_vel.yaml`

加速度制限 $a \le 1.0\,\text{m/s}^2$ の下ではボトムスピード低下のタイム損出が大きいため、コーナーボトム速度を向上させつつ直線最高速を 45.0 km/h に最適化しました。

```yaml
ref_vel_configulator:
  s0_start_straight: { ref_vel: 45.0, wp_id: 1 }
  s1_corner1:         { ref_vel: 39.5, wp_id: 60 }
  s2_straight1:       { ref_vel: 45.0, wp_id: 85 }
  s3_hairpin1:        { ref_vel: 36.5, wp_id: 110 }
  s4_straight2:       { ref_vel: 45.0, wp_id: 135 }
  s5_corner2:         { ref_vel: 37.5, wp_id: 160 }
  s6_straight3:       { ref_vel: 45.0, wp_id: 190 }
  s7_corner3:         { ref_vel: 37.5, wp_id: 210 }
  s8_chicane_early_brake: { ref_vel: 35.5, wp_id: 225 }
  s9_chicane_hairpin:     { ref_vel: 35.5, wp_id: 240 }
  s10_chicane_exit_curve: { ref_vel: 36.5, wp_id: 260 }
  s11_straight4:      { ref_vel: 45.0, wp_id: 300 }
  s12_corner4:        { ref_vel: 37.5, wp_id: 305 }
  s13_corner5:        { ref_vel: 37.5, wp_id: 325 }
  s14_final_straight: { ref_vel: 45.0, wp_id: 340 }
```

---

## 4. 自動データ解析サブシステム仕様 (Telemetry Subsystem)

`make dev` の実行（車両スポーン・制御開始）とミリ秒単位で完全に同期して起動し、走行データを集計・プロット出力するシステムです。

### 構成ファイル一覧
1. **[analyze_trajectory.py](file:///home/ci008043/workspace/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/analyze_trajectory.py)**
   - ROS 2 トピック (`/localization/kinematic_state`, `/control/command/control_cmd`, `/mpc/ref_path`) を計測。
   - `/output/YYYYMMDD-HHMMSS/trajectory_analysis.png` にプロット出力。
2. **[run_analyze_trajectory.bash](file:///home/ci008043/workspace/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/run_analyze_trajectory.bash)**
   - ソースコードディレクトリ直参照フォールバック付きシェルラッパー（未ビルド時の Launch クラッシュを回避）。
3. **[mpc.launch.xml](file:///home/ci008043/workspace/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/mpc.launch.xml)**
   - MPC起動時に解析ノードを自動同期起動。

```xml
<node pkg="multi_purpose_mpc_ros" exec="run_analyze_trajectory.bash"
      name="analyze_trajectory" output="screen">
  <param name="use_sim_time" value="$(var use_sim_time)"/>
</node>
```

### 解析出力画像 (`trajectory_analysis.png`) 構成
- **Subplot 1 (2D Driving Line)**: 参照ラインと車速カラーマップ付き実走軌跡プロット。
- **Subplot 2 (Velocity Profile)**: 距離 $s$ に対する車速推移 $v(s)$。
- **Subplot 3 (Acceleration Check)**: 加速度指令 $a_x(t)$ と 1.0 m/s² ルール超過検証。
- **Subplot 4 (Summary)**: 各周タイム、ベストタイム、平均タイム、ルールの合否判定 (PASS/FAIL)。

---

## 5. 開発・ビルド・運用手順

```bash
# 1. 実行権限の付与 (初回)
chmod +x aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/*.bash
chmod +x aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/*.py

# 2. パッケージのビルド
make autoware-build

# 3. シミュレーション起動 (完全自動同期で解析ノードもスタート)
make dev
```
