# 周回走行解析機能（Telemetry Subsystem）設計仕様書 (Design Specification)

> **対象パッケージ**: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`  
> **最終更新**: 2026-08-15  
> **対象読者**: 人間開発者 / AIエージェント  
> **目的**: 自動走行データ解析環境、周回タイム集計、SW部門競技ルール適合判定（加速度制限）、走行軌跡・速度・加速度・ステアリング安定性プロファイルの可視化、および周回短縮着目点（⭐）と制御不安定箇所（▲）の自動識別仕様の記録。

---

## 1. 全体仕様・目的概要

本設計仕様書は、Autoware Racing Kart の走行シミュレーションにおいて、走行テレメトリデータをミリ秒単位で自動収集・解析し、周回タイム計測、制御プロファイル可視化、競技ルール適合性検証、ならびに**周回時間短縮のためのボトルネック特定（⭐）**と**制御発振・不安定箇所の自動診断（▲）**を行う解析サブシステムの構成を定義します。

すべての変更は提出制限ルールに従い、`aichallenge/workspace/src/aichallenge_submit/` ディレクトリ配下にのみ配置されています。

---

## 2. 競技ルール（SW部門）適合検証仕様

解析システムは、シミュレーション走行中に以下の競技ルール適合性を監視・検証します。

| 競技ルール項目 | 制約内容 | 解析システムでの検証・判定仕様 |
|---|---|---|
| **① 加速度上限** | 最大 **1.0 m/s²** | `/control/command/control_cmd` の目標加速度 $a_x(t)$ を常時監視。1.0 m/s² を超過したフレームの有無を検出し、レポートに PASS/FAIL を出力。 |
| **② 制動下限** | 減速限界 **-1.6 m/s²** | 急制動時の加速度が規定限界範囲内にあるかを検証。 |
| **③ 提出範囲制約** | `aichallenge_submit/` 配下のみ変更可 | 解析ノード・スクリプト・Launch設定をすべて `aichallenge_submit/` 内に完結。 |

---

## 3. 自動データ解析サブシステム仕様 (Telemetry Subsystem)

`make dev` の実行（シミュレーション起動・制御開始）と完全に同期して起動し、走行データを集計・プロット出力するシステムです。

### 3.1 構成ファイル一覧
1. **[`analyze_trajectory.py`](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/analyze_trajectory.py)**
   - ROS 2 トピック (`/localization/kinematic_state`, `/control/command/control_cmd`, `/mpc/ref_path`) を計測。
   - 周回検出（スタートライン通過判定）、ラップタイム集計、最大加速度のルール検証を実施。
   - **周回短縮着目点（⭐）**および**制御不安定箇所（▲）**の自動アルゴリズム診断。
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
| `/mpc/ref_path` | `visualization_msgs/msg/MarkerArray` | 参照走行ライン $(x_{ref}, y_{ref})$ の取得・横偏差 $e_y$ 算出 |

---

## 4. 特徴量抽出と自動診断アルゴリズム仕様

### 4.1 ⭐ 周回時間短縮のための着目箇所（Time-Loss / Optimization Focus）
人間開発者がタイムアタックの改善ポイントを一目で把握できるよう、以下のアルゴリズムでタイムロス要因を特定します。

- **過剰減速コーナー（Slow Corner）**:
  - 車速プロファイルの極小値（ボトムスピード）のうち $v < 35.0\,\text{km/h}$ の区間を抽出。
- **脱出加速遅延（Sluggish Exit Accel）**:
  - 直線・コーナー脱出時（$|\delta| < 0.08\,\text{rad}$）に目標加速可能であるにもかかわらず $a_{\text{cmd}} < 0.5\,\text{m/s}^2$ に留まっている区間を抽出。
- **走行ライン膨らみ（Wide Line）**:
  - 目標パスからの横偏差 $e_y = \min_j \sqrt{(x - x_{ref,j})^2 + (y - y_{ref,j})^2}$ が $0.45\,\text{m}$ を超えて膨らんでいるピーク位置を抽出。

### 4.2 ▲ 制御不安定箇所（Control Instability / Chatter）
制御破綻やステアリングのバタつきを早期に発見するため、以下の不安定挙動を検出します。

- **ステアリングハンチング（Steering Oscillation）**:
  - 操舵角速度 $|\dot{\delta}| = |\frac{\Delta \delta}{\Delta t}| > 0.6\,\text{rad/s}$ かつ近傍ウィンドウ内で正負反転（切り返し）が頻発する区間を抽出。
- **加速度指令チャタリング（Accel Chatter）**:
  - 加速度変化率 $\dot{a}$ の分散が閾値を超える急激なスロットル/ブレーキの小刻みな変動を抽出。

### 4.3 識別デザイン・高視認性仕様（オクルージョン対策）

グラフ上の波形や走行ラインをテキストボックスで覆い隠さないよう、**非極大値抑制（NMS）**により重要度の高い上位各4箇所（Top-K）に厳選し、グラフ上はコンパクトな**インデックスバッジ（`[T1]..[T4]`, `[U1]..[U4]`）**で表示します。詳細情報（速度・角速度・対策案）はすべて右下のダッシュボード（Subplot 4）に集約連携されます。

| 診断分類 | シンボル | バッジ表記 | 配色 | 目的 |
|---|---|---|---|---|
| **周回短縮着目点** | `⭐` (Star) | `[T1]`, `[T2]`, `[T3]`, `[T4]` | Cyan (#00FFFF) | 最も改善効果が高いタイムロス箇所（過剰減速・加速遅延・ラインロス）を特定 |
| **制御不安定箇所** | `▲` (Triangle) | `[U1]`, `[U2]`, `[U3]`, `[U4]` | Red (#DC2626) | 最も顕著なステアリングハンチング・制御チャタリング箇所を特定 |

---

## 5. 解析出力画像 (`trajectory_analysis.png`) 4画面構成

- **Subplot 1 (2D Driving Line & Track Boundaries vs Reference)**:
  - **コース外観（Track Boundaries / Walls）**: Occupancy Grid Map（またはトラック境界ポリゴン）から内壁・外壁を濃いスレートグレー（`#334155`）の輪郭線で描画。
  - **目標ライン（Reference Raceline）**: `traj_mincurv.csv` の全周352点を鮮明なパープル/インディゴ破線（`#6366f1` / `--`）で閉曲線としてクッキリ描画（背景同化を完全解消）。
  - **スタート/フィニッシュライン（Start/Finish Line）**: メインストレートの実位置（`X=89633.29, Y=43127.57`）に緑色スクエアマーカー（`■`）とバッジを配置。
  - **実走行軌跡（Actual Logged Driven Path）**: `LineCollection` により車速グラデーション（`jet` カラーマップ）の連続太線（`linewidth=2.8`）で描画。ステップ間距離が物理的に連続している実走行区間（$\Delta d < 1.5\,\text{m}$）のみをセグメント化して描画し、初期配置や周回リセット時の**架空の壁貫通直線を完全排除**。
  - ⭐（タイムロス箇所 `T1..T4`）と ▲（制御不安定箇所 `U1..U4`）をコンパクトバッジでオーバーレイ表示。
- **Subplot 2 (Velocity Profile & Optimization Points)**:
  - **デュアル X 軸**: 下軸に走破距離 $s$ [m]、上軸に経過時間 $t$ [s] を完全連動表示。
  - 距離 $s$ [m] に対する車速推移 $v(s)$ および横偏差 $e_y(s)$。
  - ボトムスピード低下点や加速遅延区間を `T1..T4` バッジでハイライト。
- **Subplot 3 (Control Stability & Rule Compliance)**:
  - **デュアル X 軸**: 下軸に走破距離 $s$ [m]、上軸に経過時間 $t$ [s] を完全連動表示（Subplot 2 と上下で位置・時間完全同期）。
  - 加速度指令 $a_x(s)$ とステアリング角速度 $|\dot{\delta}(s)|$。
  - 半透明赤色シェード（`axvspan`）と `U1..U4` バッジにより、発振区間をコース位置および時間基準で明示。
- **Subplot 4 (Diagnostics Dashboard & Lap Summary)**:
  - **コース・リファレンス基本指標**: 全周基準走行距離（約 351.7 m / 352 ウェイポイント）、スタート/フィニッシュ座標。
  - 識別バッジ凡例（100% ASCII英語表記・文字化けゼロ）。
  - 各周ラップタイム、ベストタイム、平均タイム。
  - **タイム短縮改善ポイント Top 4 (`T1..T4`)**（発生距離 $s$、車速、対策アドバイス）。
  - **制御不安定診断 Top 4 (`U1..U4`)**（発生距離 $s$、角速度、**発振周波数 $f$ [Hz]、継続時間 $\Delta t$ [s]**、MPC パラメータチューニング推奨値）。
  - 加速度ルール適合判定 (PASS/FAIL)。

---

## 6. 開発・ビルド・運用手順

```bash
# 1. パッケージのビルド
make autoware-build

# 2. シミュレーション起動 (自動同期で解析ノードもスタート)
make dev
```

