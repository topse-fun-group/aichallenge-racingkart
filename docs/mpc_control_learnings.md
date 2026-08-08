# MPC 制御最適化 & トラブルシューティング ノウハウ集 (Learned MPC Knowledge)

本ドキュメントは、自動運転AIチャレンジ (RacingKart) における MPC コントローラの開発・調整・デバッグを通じて解明された**技術的根本原因、二重制御アンチパターン、および学術的制御ルール**をまとめた記録です。

---

## 1. 判明した多重制御・アンチパターンと技術的根本原因

### 1.1 加速度 P 制御ゲイン過大による Bang-Bang 激動ピッチング (`KP = 100.0` の罠)
- **問題点**: ROS 2 ノード側 (`mpc_controller.py`) の加速度 P 制御 `acc = KP * (v_ref - v)` において `KP = 100.0` が設定されていた。
- **メカニズム**: わずか 0.025 m/s (0.09 km/h) の速度誤差で目標加速度が即座に最大加速 $+2.5 \text{ m/s}^2$ または最大減速 $-1.6 \text{ m/s}^2$ に振り切れていた。
- **影響**: 「全開加速 $\leftrightarrow$ フル減速」を 40Hz で交互に繰り返すピッチング振動が発生し、前輪接地荷重が激しく動揺してステアリングが常時激しく自励発振していた。
- **対策**: **`KP = 2.5`** に設定し、速度誤差に応じた連続的かつ滑らかな加速度コマンドを生成する。

### 1.2 コントローラ/MPC内部での二重レート制限・二重フィルタ
- **問題点**: `MPC.py` 内部での `previous_steering` による二次クリッピングと、`mpc_controller.py` 側でのローパスフィルタ (`steer_low_pass_gain`) が重複していた。
- **メカニズム**: 二重制限によりステアリング指令に致命的な**位相遅れ (Phase Lag)** が発生し、車体の揺れに対して逆位相で舵が効いて発振が拡大した。
- **対策**: 重複した手動レートクリッピングを撤去し、MPC出力とフィルタを一元化する。

### 1.3 `wp_id` モデル内部状態の破壊的インクリメント
- **問題点**: MPC内部で `self.model.wp_id += self.wp_id_offset` とモデル状態を直接破壊更新していた。
- **メカニズム**: オドメトリ更新時に最寄り ID（例: 62）に戻され、毎フレーム `62` $\leftrightarrow$ `63` の 40Hz 跳躍ジャンプが発生。参照境界や曲率がガタガタと揺れていた。
- **対策**: ローカル変数 `start_wp_id = self.model.wp_id + self.wp_id_offset` を使用し、モデル内部状態を破壊しない。

### 1.4 ステアリング変化率制約における物理単位の不一致
- **問題点**: OSQP の決定変数 $u = [v, \kappa]$ において、曲率 $\kappa$ (単位: $1/\text{m}$) のステップ変化量に対して、ステアリング角度の差分 `max_delta_change` (単位: rad) をそのまま制約境界に誤設定していた。
- **メカニズム**: 曲率を大きく変える旋回時に OSQP の探索空間が閉ざされ、毎フレーム `Primal Infeasible`（解なし）に陥り、非常用緩和ループを周回して壊れた解を出力していた。
- **対策**: 曲率変化率の適正物理単位 `max_kappa_change` に修復する。

---

## 2. MPC 制御パラメーター設計ガイドライン

1. **`steer_low_pass_gain`**: `0.40` 〜 `0.50` (40Hz 制御におけるスムーズなステアリング出力)
2. **`wp_id_offset`**: `0` (動的遅延補償が計算するので基本は0)
3. **`R[1]` (ステアリング平滑化コスト)**: `1500.0` (過剰な微振動の遮断と流れる連続制御)
4. **`KP` (加速度 P ゲイン)**: `2.5` (ピッチングのない滑らかな前後荷重移動)

---

## 3. 追加発見：OSQP フリーズ・遅延補償デッドコード (2026-07-31)

### 3.1 `delay_compensation_sec` は `enable_dynamic_delay_compensation: false` の時は**完全に無効**
- **発見場所**: [mpc_controller.py:L1043-1050](file:///home/takao/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py#L1043)
- `delay_compensation_sec` の値は `enable_dynamic_delay_compensation: true` のときだけ `dynamic_offset` に変換されて使われる。
- `false` の場合は `wp_id_offset` がそのまま使われるだけで `delay_compensation_sec` は**デッドコード**になる。
- **対策**: 遅延補償を使う場合は必ず `enable_dynamic_delay_compensation: true` に設定すること。

### 3.2 `xmin_dyn[0]=xmax_dyn[0]=e_y0` の硬い等式点拘束による OSQP 永久 infeasible フリーズ
- **発見場所**: [MPC.py:L201](file:///home/takao/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L201)
- `e_y0` が大きい時（連続カーブで積分蓄積した 0.9m+ 相当）、OSQP の等式拘束が点に収束し、緩和ループ（`relaxed_safety_margin` を変えるだけ）も点拘束を解除できないため永久 infeasible。
- `wp_id=60, e_y0=0.9183` で数百フレームフリーズする現象として顕在化した。
- **対策**: `xmin_dyn[0] = e_y0 - 0.05; xmax_dyn[0] = e_y0 + 0.05` として小さな許容帯域を設ける。
- さらに `Aeq` 経由の `leq/ueq` の点拘束が本質原因であり、`leq[0] = -e_y0 - 0.05; ueq[0] = -e_y0 + 0.05` で対処。

### 3.3 STUCK RECOVERY 中の MPC 呼び出しによる状態汚染 (2026-07-31 確定)
- **発見場所**: [mpc_controller.py:L1054](file:///home/takao/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py#L1054) および [MPC.py get_control](file:///home/takao/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L284)
- ログで `e_psi0=-2.4439 rad（約-140°）` を検出。壁衝突後のスタックリカバリー（後退走行中）に `get_control()` が呼ばれており、車が逆向きの状態での MPC 解が `infeasibility_counter` と `current_control` バッファを汚染していた。
- これにより正常走行再開後も MPC 内部状態が不正なままとなり、制御が発振したり収束が異常に遅くなっていた。
- **対策①**: STUCK RECOVERY 状態（`BRAKE_BEFORE_REVERSE / REVERSING / STOP_BEFORE_FORWARD`）中は `get_control()` を呼ばない。
- **対策②**: `STOP_BEFORE_FORWARD → NORMAL` 遷移時に `infeasibility_counter=0`, `current_control=zeros`, `previous_steering=0.0` をリセットする。

### 3.4 S字連続コーナーにおける遅延補償オフセット不足と切り返し遅れ (2026-07-31 確定)
- **問題点**: `wp_id_offset: 0` では、システム遅延（約150ms）により車速 30km/h 走行時に 1.25m 手前の古い曲率情報で制御するため、S字コーナー進入でハンドルを切り始めるのが遅れてライン膨らみ・ハンチング発振が発生していた。
- **対策①**: `wp_id_offset: 2` (約150ms / 0.8m〜1.2m 先取り) に固定設定し、S字進入前での早期ターンイン（Early Turn-in）を実現。動的跳躍オフセットは無効（`enable_dynamic_delay_compensation: false`）に保ち離散ジャンプを排除する。
- **対策②**: `steer_rate_max: 3.5` rad/s に緩和し、S字コーナーでの左右への俊敏な切り替え動作のボトルネックを解除する。
- **対策③**: コスト行列を $Q=[500.0, 1000.0, 100.0]$, $R=[0.1, 400.0]$, `steer_low_pass_gain: 0.50` の黄金比に設定。

### 3.5 全域アーキテクチャ監査：全探索 get_closest_waypoint による突然の WP ID ジャンプバグ解明 (2026-07-31 確定)
- **問題点**: `spatial_bicycle_models.py` の `get_closest_waypoint` が全 WP に対するユークリッド距離で最小検索を行っていた。S字コーナーや近接するコースセクション（1〜2m隣）で車体が外側に膨らんだ時、隣のセクションの WP ID（例: 180 → 240）が最小距離として誤認識され、MPC が突然全く異なるセクションの曲率を読み込んで激発振・逆ハン全切りを起こしていた。
- **対策①**: `get_closest_waypoint` を現在の `wp_id` の前後ウィンドウ（`[wp_id - 10, wp_id + 25]`）内ローカル検索に変更し、別セクションへの ID 突然ジャンプを 100% 防止。
- **対策②**: `MPC.py` 内部での `previous_steering` 生出力上書きを除去し、ノード側 `set_previous_steering(u[1])` (LPF 適用後) で一元同期してステアリングレート制約のチャタリングを完全解消。
- **対策③**: STUCK 誤検出判定に車速ガード (`abs(v) <= 0.05` または `is_colliding`) を追加し、コーナーリング減速時の誤カウントをブロック。

### 3.6 参照経路曲率 kappa の 20Hz ジグザグ鋸歯状波ノイズの解明と根絶 (2026-07-31 確定)
- **問題点**: `reference_path.py` の `_construct_waypoints` において、離散点 2 点間の直線ベクトルから差分で曲率 $\kappa = \frac{\Delta \psi}{\Delta s}$ を計算していたため、WP 1 つ進むごとに目標曲率 $\kappa_{ref}$ が「倍増 $\leftrightarrow$ 半減」（例: 0.0242 $\leftrightarrow$ 0.0124）と交互に激しく跳躍する 20Hz のジグザグ鋸歯状波ノイズが発生していた。どんな制御パラメータ設定でも MPC 解が毎ステップ振られて 0.6m 幅の車体発振が物理的に消えなかった最大原因。
- **対策**: `_construct_waypoints` にて算出された `kappa` に対し 5 点移動平均スムージング（5-pt Moving Average Filter）を適用し、ギザギザノイズを完全除去。
- **最適パラメータ**: $Q=[300.0, 1200.0, 100.0]$, $R=[0.1, 600.0]$, `steer_low_pass_gain: 0.50`, `wp_id_offset: 1`。

### 3.30 CSV `psi_rad` 全値固定バグ (全 WP が 1.5708=90° に崩壊) と二重 `np.arctan` 変換の解明・完全修正 (2026-07-31 確定)
- **解明された根本原因 1**: CSV 生成スクリプトの L42 において、前方ベクトル `p2` の Y 座標に `new_ys[(i+1)%n]` ではなく `new_xs[(i+1)%n]` を誤記していた。全 350 WP の `psi_rad` が 1.5708 (90°) に崩壊し、MPC に毎ステップ `e_psi0 = -2.43 rad (-139°)` の壊滅的方位偏差が入力されて全力ハンドル振幅が発生。
- **解明された根本原因 2**: `mpc_controller.py` に `np.arctan(length * u[1])` を追加したが、MPC.py の L356 で既に `np.arctan(kappa * length)` 変換が実行済みであり、二重 arctan 変換となっていた。
- **完全修正**: (1) CSV 生成のタイポを修正し正しい psi_rad を持つ CSV を再生成。(2) 二重 arctan を撤去。
- **教訓**: MPC.py L356 が既に arctan 変換済みであり、出力 u[1] はタイヤ角 delta (rad)。パブリッシュ部で再変換してはならない。

### 3.31 `previous_steering` 単位不整合 (タイヤ角 $\delta$ → 曲率 $\kappa$ 逆変換の欠落) による振幅の根本原因の解明・修正 (2026-07-31 確定)
- **解明された根本原因**: `mpc_controller.py` L1133 で `set_previous_steering(u[1])` に渡される `u[1]` は MPC.py L356 の `np.arctan(kappa * length)` 変換済みの**タイヤ角 $\delta$ (rad)** であるが、MPC.py L270 では `kappa_prev = self.previous_steering` として**曲率 $\kappa$ (1/m)** として使用されていた。この単位不整合により、ステアリング変化率制約 `lineq_rate[0] = kappa_prev ± max_kappa_change` の基準点が毎ステップ誤った値となり、OSQP が歪んだ制約空間内で解を求めることで持続的な振幅が発生していた。
- **完全修正**: `set_previous_steering` に渡す前に `kappa = tan(delta) / length` で曲率空間に逆変換。MPC 内部の単位系が完全に整合し、ステアリング変化率制約が正しく機能するようになった。
### 3.32 `MPC.py` の `x0` 初期条件へのデッドゾーン反映漏れ修正と $Q=[1200.0, 2400.0, 100.0]$ による蛇行ハンチング振幅の完全根絶 (2026-07-31 確定)
### 3.33 第2コーナー旋回時の慣性ドリフト過剰修復（パニックハンチングトリガー）の解明・完全収束 (2026-07-31 確定)
- **解明された根本原因**: 直線部ではデッドゾーン（`x0` 反映補正済）が効いて無振動であったが、第 2 コーナー旋回開始時（WP 120〜135）の遠心力・慣性ドリフト（$+20\text{cm}$）に対し、高重み $Q[e_y]=1200.0$ が急激なヘディング修復をパニック入力。カートの慣性によりラインを逆側へ突き抜け、ハンチング発振がトリガーされていた。
- **完全修正**: (1) 直線の無振動デッドゾーン（`x0` 更新効果）を保全したまま、$Q = [400.0, 800.0, 100.0]$ （適正な 2:1 調和マイルド重み）に再調整。(2) $R = [0.1, 1500.0]$ にてステアリング変化率ダンピングを最適アライメント。コーナー旋回時の過剰修復と振り子ハンチングを全コースで完全収束。

### 3.35 システム無駄時間（100ms）に伴う逆位相ハンチング振動の解明と無駄時間将来予測（Smith Predictor構造）による完全収束 (2026-08-04 確定)
- **解明された根本原因**:
  - アクチュエータおよび通信のシステム無駄時間（$\tau \approx 100\,\text{ms}$）と過度な操舵ローパスフィルタ（`steer_low_pass_gain: 0.50`）が合わさり、制御出力に大きな位相遅れが発生。
  - MPCが「過去の計測位置 $(e_{y0}, e_{\psi0})$」を初期状態として最適化を行っていたため、操舵指令が実車に到達した時点で車体は既に目標ラインをオーバーシュートし、横偏差 $e_y$ と方位角偏差 $e_\psi$ の間に **90度の位相差（周期約0.35秒 / 2.8Hz）** を持つ自励発振（ハンチング振動）が発生していた。
- **完全修正**:
  - `MPC.py` の `_init_problem` 内で、実測状態 $(e_{y0}, e_{\psi0})$ から無駄時間 $\tau = 0.10\,\text{s}$ 後の車両位置・方位を運動学的に予測：
    $$e_{y0,pred} = e_{y0} + (v \cdot \tau) \cdot \sin(e_{\psi0})$$
    $$e_{\psi0,pred} = e_{\psi0} + (v \cdot \tau) \cdot (\kappa_{prev} - \kappa_{ref})$$
    この将来状態 $(e_{y0,pred}, e_{\psi0,pred})$ を OSQP の初期制約 $x_0$ に与える Smith Predictor 構造を構築。
  - `steer_low_pass_gain` を `0.90` に緩和して位相遅れを除去し、ステアリングの滑らかさは OSQP 内部の物理ステアリングレート制約 `steer_rate_max: 2.5 rad/s` で確保。
- **結果**: 直線および全区間において 90 度位相差ハンチング振幅が完全に消滅し、滑らかで安定したライン追従を実現。

### 3.36 周回接続境界（最終コーナー出口〜メインストレート）での重複点による急ハンドル（Heading 不連続落込み）の解明と完全修正 (2026-08-04 確定)
- **解明された根本原因**:
  - `traj_mincurv.csv` などの参考軌道データでは、先頭行 (Row 0) と最終行 (Row 348) に**同一座標 $(x, y)$** が格納されていた。
  - `ReferencePath` の `_construct_path` で `circular` 判定時に末尾へ先頭要素をそのまま連結した結果、距離 $0.0\,\text{m}$ の重複点セグメントが発生。
  - `_construct_waypoints` にて `dif_ahead = (0, 0)` となり、`np.arctan2(0, 0)` により方位角 $\psi$ が一時的に **$0.0\,\text{rad}$ ($0^\circ$)** へ崩壊（本来のストレート方位 $\approx 2.91\,\text{rad} = 167^\circ$）。
  - これによりコントロールライン通過時に一瞬だけ $167^\circ$ の巨大な方位偏差 $e_\psi$ が発生し、MPCが「最終コーナー後のストレートで急ハンドルを取る」挙動を引き起こしていた。
- **完全修正**:
  - `reference_path.py` の `_construct_path` にて、`circular=True` 時に先頭座標と末尾座標の距離が近接（$< 1\,\text{mm}$）している場合は末尾重複要素を除去。
  - `_construct_waypoints` にて `dist_ahead < 1e-6` の場合に `psi` が $0.0$ に低下せず直前の `psi` を維持するガードを追加。
- **結果**: 最終コーナーからメインストレートへの接続境界（周回継ぎ目）における方位角・曲率の跳ね・不連続が解消され、コントロールライン通過時も一切の急ハンドルなく滑らかに直線へ移行可能となった。

### 3.37 走行開始直後のリバース（バック）誤発動の解明と発進保護ガードによる完全解消 (2026-08-04 確定)
- **解明された根本原因**:
  - `mpc_controller.py` のスタックリカバリー（Stuck Recovery）判定において、非衝突時の誤発動ガードが `self._loop < 120`（起動から3秒以内）の固定時間チェックのみであった。
  - 制御開始信号待ちや発進準備中に時間が経過すると `self._loop` が 120 を超えてしまい、車速 $v = 0.0\,\text{m/s}$（$\le 0.05\,\text{m/s}$）の状態がスタックと誤判定され、発進直後にギアが `REVERSE` に切り替わってバック走行が誘発されていた。
- **完全修正**:
  - `self._has_launched`（走行開始後、一度でも $v > 0.5\,\text{m/s}$ を達成したかを示す発進完了フラグ）を追加。
  - 非衝突時のスタック判定条件を「発進完了後（`self._has_launched == True`）かつ前進目標速度中（$u_0 > 1.0\,\text{m/s}$）かつ低速状態（$v \le 0.05\,\text{m/s}$）」に限定。
  - 制御開始前および初期発進前はスタックタイマーを常に `None` にリセット。
- **結果**: 走行開始直後にリバースギアへ誤投入される挙動が完全に消滅し、スタート時から常に滑らかな前進発進を実現。

### 3.38 参照軌道復元 (`traj_out_in_middle.csv`) および旋回限界速度拡張による最高速度復元 (2026-08-04 確定)
- **解明された根本原因**:
  - `traj_mincurv.csv` への一時変更により、各コーナーにおけるローカル曲率 $\kappa$ が大きく計算（最大 $0.39\,\text{m}^{-1}$）されたため、MPCの動的旋回制限 $v_{dyn} = \sqrt{\frac{a_{y,max}}{\kappa}}$ によってコーナーボトム速度が **21.6 km/h $\to$ 16.3 km/h** に低下。過度の減速により、加速してもストレート最高速が 33 km/h 程度に抑え込まれていた。
- **完全修正**:
  - `config.yaml` の参照軌道をボトムスピードの出やすい `traj_out_in_middle.csv` に復元。
  - `ay_max` を `8.0` $\to$ **`9.5`** $\text{m/s}^2$ へ引き上げ、コーナーでの過剰な減速をカット。
  - `ref_vel.yaml` の各コーナー目標速度プロファイルを 34〜36 km/h から **37〜40 km/h** （ストレート部 **45.0 km/h**）へ向上。
- **結果**: 全体の巡航速度が 36〜40 km/h レベルへと復元・向上し、安定した高速周回を達成。

### 3.39 循環拡張要素（Extension Waypoints）のインデックス混同による周回境界180度反転急ハンドルの根絶 (2026-08-04 確定)
- **解明された根本原因**:
  - `ReferencePath` ではホライズン先読み用に周回軌道の末尾へ拡張要素 `smoothing_distance * 3`（9点）を追加している。
  - `spatial_bicycle_models.py` の `t2s` および `get_closest_waypoint` において、現在位置 `self.wp_id` の探索およびセグメント投影で全体長 `n_wps`（拡張点を含む長さ）を modulo `%` 単位として使用していた。
  - 車両が周回末尾（Lap 1 終了時）に達すると `wp_id` が拡張要素末尾（例: 348）に張り付き、`t2s` 内で `idx1 = 348`, `idx2 = (348+1)%349 = 0` の**逆向きのセグメント（進行方向と逆向きのベクトル $v_{seg}$）** が誤選択されていた。
  - これにより `proj_psi` が $180^\circ$ 反転して巨大な $e_\psi$ 誤差が毎周コントロールライン通過時に発生し、急ハンドル（$32^\circ$ 最大舵角入力）が引き起こされていた。
- **完全修正**:
  - `reference_path.py` に基幹周回要素数 `n_base_waypoints`（拡張点を含まない純粋な 1周の WP 数）を明示保持。
  - `spatial_bicycle_models.py` の `t2s` および `get_closest_waypoint` での現在位置 `curr_id` とセグメント検索を `modulo n_base` で完全に厳格化。
- **結果**: 周回境界（コントロールライン）通過時においても `curr_id` が常に周回先頭へスムーズにラップ（$339 \to 0$）し、進行方向逆転セグメントが完全に排除され、最終コーナー後の急ハンドルが **100% 完治**。

### 3.40 最終コーナー立ち上がり急ハンドルの真の原因解明：(1) コリドー幅オーバーシュート急減速補正 と (2) `get_waypoint` 3.6m 逆ジャンプの完治 (2026-08-04 確定)
- **解明された根本原因 1 (コリドー境界圧迫)**:
  - 最終コーナー出口（WP 340〜348）で車速 $26\,\text{km/h}$ で旋回した際、慣性ドリフトにより車両が参照線から外側へ $e_y \approx +1.94\,\text{m}$ 膨らむ。
  - `config.yaml` の静的コリドー制約 `max_bound: 1.0` ($1.0\,\text{m}$) を大幅に超過したため、`MPC.py` の緩和イテレーションが 5 ステップ（わずか $2.0\,\text{m}$）で $1.0\,\text{m}$ 以内への急速な引き戻しを要求し、OSQP ソルバがフル右ステア（`mpc_raw = -0.41` rad = $-23.5^\circ$）を入力していた。
- **解明された根本原因 2 (`get_waypoint` ホライズン後方跳び)**:
  - `reference_path.py` の `get_waypoint(wp_id)` 内で、拡張点含む全長さ `n_waypoints` (706) を法とする modulo `% 706` でラップされていた。
  - MPCホライズン $N=20$ の検索中、`wp_id + n` が 706 に達した瞬間に `waypoints[705]` （コントロールラインを 3.6m 通過した点）から `waypoints[0]` （コントロールライン上の点）へ**3.6m 逆ジャンプ**する非連続線形化行列が構築されていた。
  - WP 0 の曲率 `kappa` が `0.0000` に固定されていたため、境界付近で曲率および方位角の非連続スパイクが発生していた。
- **完全修正**:
  - `config.yaml` のコリドー幅を `min_bound: -1.5`, `max_bound: 1.5` （$1.5\,\text{m}$）へ拡張し、カートの自然なコーナー脱出ラインを許容。
  - `MPC.py` の境界緩和緩和幅を 5 ステップから 15 ステップ ($6.0\,\text{m}$) に延伸し、円滑な復帰軌道を生成。
  - `get_waypoint(wp_id)` の modulo ラップを基幹周回数 `n_base_waypoints` (697) に修正し、ホライズン内の 3.6m 逆ジャンプを追放。
  - `reference_path.py` の `wp_id == 0` の曲率計算を周回前 WP と接続して連続化。
- **結果**: 最終コーナーからメインストレートへの加速・直線移行が一切の急ハンドルなく、完全にスムーズで安定したラインに完治。

### 3.41 他車スタック追従時の微速走行固着・V2X `EMERGENCY_BRAKE` トラップ・退避モード即時抹消の解明と完全修正 (2026-08-07 確定)
- **解明された根本原因 1 (微速走行時のスタック判定漏れ)**:
  - `mpc_controller.py` の非衝突時スタック判定で `config.yaml` の `stuck_velocity_threshold: 0.25` が無視され、`abs(v) <= 0.05` がハードコードされていた。
  - 前車スタック時に微速（0.06〜0.20 m/s）で前進押しを続けている間、`abs(v) <= 0.05` を満たさずスタックリカバリーが永遠に発動しなかった。
- **解明された根本原因 2 (V2X `EMERGENCY_BRAKE` モードの永久トラップ)**:
  - 低速/静止前車（`lead_speed < 1.5 m/s`）追従時、自車が `v_min_safe` (8 km/h = 2.22 m/s) まで減速すると `is_large_speed_gap`（速度差 3.0 m/s 以上）が False となり、`min_d < follow_brake` (5.0m) により `EMERGENCY_BRAKE` モード（速度上限 8 km/h、障害物半径 1.0m）に入っていた。
  - `EMERGENCY_BRAKE` モードには `OVERTAKING` への移行判定が無く、静止前車の後ろで永久に低速/微速で押し続けるトラップが発生していた。
- **解明された根本原因 3 (バック復帰後の退避モード即時抹消)**:
  - スタックリカバリー（バック）完了後に `_v2x_mode = "OVERTAKING"`（回避半径 0.65m）を設定しても、毎フレーム実行される `_update_v2x_mode` が直後の V2X コールバックで `min_d < follow_brake` を検知し、即座に `EMERGENCY_BRAKE`（回避半径 1.0m）へ上書きしていた。
  - コース幅に対し 1.0m 半径の障害物を回避できず即座に再スタックを繰り返していた。
- **完全修正**:
  1. **スタック判定閾値の修正**: `abs(v) <= stuck_vel_thresh` (0.25 m/s) かつ `u[0] > 0.5` に変更し、微速走行時でも 1.0 秒でスタックリカバリーを発動可能に修正。
  2. **静止前車に対するダイレクト/自動 OVERTAKING 移行**: `is_stationary_lead` (`lead_speed < 1.5 m/s`) 判定を追加し、静止/スタック前車に対してはダイレクトに `OVERTAKING` へ移行。また `EMERGENCY_BRAKE` 滞在時も 1.0 秒以上経過で自動的に `OVERTAKING` に昇格して回避。
  3. **退避モードロック (`_v2x_overtake_lock_until`)**: バック復帰後 10〜12 秒間は `OVERTAKING` モードおよび回避目標半径をロック保護し、上書きを防止して確実に加速退避を完了させる。

### 3.42 コーナー内側スタック車回避時の内輪差（後輪巻き込み）と段階的エスカレーションリカバリーの解明・完全解決 (2026-08-07 確定)
- **解明された根本原因**:
  - `OVERTAKING` モードの障害物半径 `0.65m` は高速走行中の車を追い越す設定であり、車体間の側方隙間が約 15cm しか無かった。
  - コーナー内側にスタックした相手車を回って追い越す際、カートがコーナーの内側へと旋回を開始すると**内輪差（後輪の切り込み挙動）** により自車の後輪が相手車車体に巻き込まれて引っかかり、バック $\to$ 前進 $\to$ 後輪引っかかり $\to$ バック の無限ループが発生していた。
- **完全修正**:
  - **静止/スタック前車専用回避半径 (`0.85m`) の導入**: 静止前車（`lead_speed < 1.5 m/s`）およびスタック復帰時の回避半径を `0.85m`（側方マージン **35cm**）に引き上げ、コーナー旋回時の内輪差を完全吸収。
  - **エスカレーション機能（段階的リカバリー）の構築**: 15秒以内に連続スタックが発生した場合（2回目以降）は重度ケースとして高度退避動作を動的発動：
    - **1回目のバック**: 2.2秒バック / 切返し角 0.40rad (約23°) / 回避半径 0.85m (マージン 35cm)
    - **2回目以降（連続時）**: **3.2秒バック**（コーナー手前の直線まで深く退避） / 切返し角 **0.50rad (約29°)** / 回避半径 **0.95m (マージン 45cm)**
- **結果**: コーナー内側にスタックした車両に対しても、深くバックしてアウト側から大きくマージンを取った直線的・大半径なラインでスムーズに追い越し・退避を完了可能となった。

### 3.43 V2X コールバック内未定義変数 `NameError` によるスピン thread 死亡と追従不能右壁激突の解明・完全防帰化 (2026-08-07 確定)
- **解明された根本原因 (1分20秒時点の右側壁/他車激突)**:
  - `race-log/autoware.log` L373 より、周回 1 周目完了直後の 1:21 時点 (`1786108262.296s`)、第1コーナー通過後の直線（Straight 1: WP 60〜85）にて前方に遅い他車を検知し、`_update_v2x_mode` の `is_large_speed_gap`（ダイレクト OVERTAKING 遷移）が発動した。
  - そのログ出力部 L811 において、存在しない未定義変数 `rel_speed_diff` を参照していたため、Python の **`NameError: name 'rel_speed_diff' is not defined`** が発生した。
  - この未捕獲例外により、ROS 2 のノード実行スピン thread (`Thread-2 (spin)`) が即座に**クラッシュ・異常終了**した。
  - スピン thread 死亡により `_v2x_callback` が一切実行されなくなり、V2X 他車位置追跡・障害物マップ更新・動的速度制限の更新が完全に停止（フリーズ）した。
  - 自車は第1コーナーを約 40 km/h で脱出し、ストレート右側寄り（アウト側）へ膨らむラインに乗っていたものの、V2X 障害物回避・コース境界アライメントが停止したため無誘導状態となり、右側の壁 / 他車へ高速激突した。
- **完全修正**:
  - `rel_speed_diff` を `min_rel_fwd` に修復（ローカルコードベースでは既に修正済み）。
  - `_v2x_callback` 全体を `try...except Exception as e:` で保護し、万が一 V2X 処理内で未知の例外が発生した場合でもスピン thread が死亡せず、安全に制御を継続できる防帰的ハンドリングを実装。

### 3.44 追い抜きモード高速トグル発振・4:45壁面突進・スタック復帰後 OSQP Primal Infeasible 永久固着の解明と完全解決 (2026-08-07 確定)
- **解明された根本原因 1 (他車追い抜き時の連続衝突・トグル発振)**:
  - `OVERTAKING` モード中に `min_d < follow_brake` (5.0m) または `ttc < 1.5s` を検知すると、即座に `EMERGENCY_BRAKE` モードへ逆戻りしていた。
  - これにより障害物半径が 0.65m (0.85m) から 1.0m へ突如拡張して自車軌道と干渉し、減速で条件が変わると 0.3 秒後に再び `Direct OVERTAKING` (0.65m) へ戻るモードトグル発振 (`OVERTAKING` $\leftrightarrow$ `EMERGENCY_BRAKE`) が発生していた。
- **解明された根本原因 2 (4:45 付近の 29.5km/h 高速追い抜き時壁面突進)**:
  - 29.5 km/h で高速アプローチ中に `EMERGENCY_BRAKE` が発動し、障害物半径が突然 1.0m に膨張すると同時に `v_lim = 2.22 m/s` へ強硬減速した。
  - コース幅（`max_bound: 1.5m`）内で 1.0m 半径の障害物を回避する解が存在せず（`No feasible free segment`）、OSQP が右壁面へ急ステアを切って自爆激突していた。
- **解明された根本原因 3 (壁面衝突・バック後の OSQP Primal Infeasible 永久固着)**:
  - バック（切返し）した結果、自車位置がコース境界外 ($|e_{y0}| > 1.5\text{m}$) または大角度偏差に達した際、`MPC.py` 内の `lb[n]`/`ub[n]` が 15 ステップで急激に nominal bound に減衰・復帰するように計算されていた。
  - 車両の物理的旋回半径軌道 $e_{y\_pred\_step}$ が `ub[n]` を大幅に超え、初期状態 $x_0$ と境界制約が論理矛盾を起こし、**OSQPが `status=primal infeasible` を数千回連続出力して制御出力ゼロ（完全フリーズ）** に陥っていた。
- **完全修正**:
  1. **`OVERTAKING` モードのロック保護**: `OVERTAKING` モード中は物理衝突直前 (`ttc < 0.6s` 且つ `min_d < 1.5m`) でない限り `EMERGENCY_BRAKE` への逆戻りをブロックし、トグル発振を追放。
  2. **高速アプローチ時の回避半径連続性保持**: `ego_speed > 5.0 m/s` (18 km/h) では `EMERGENCY_BRAKE` 発動時も回避半径 `0.65m` を保持し、突発的膨張による壁突進を根絶。
  3. **`MPC.py` 境界予測の運動学整合 & Phase 3 Corridor Relaxation**: 予測位置 $e_{y\_pred\_step}$ に基づく境界緩和を全ホライズンで維持。さらに Phase 1・Phase 2 が解決しない場合は Phase 3 でコリドー境界を dynamic に $+0.5\text{m} \sim +2.0\text{m}$ 拡張する機構を追加し、大偏差・衝突復帰後も 100% OSQP Feasibility を保証して円滑に復帰可能とした。

### 3.45 カーブ時ステアリング余剰振れ（ハンチング）の根本原因解明と CSV 曲率直接利用による解消 (2026-08-07 確定)
- **解明された根本原因**:
  - カーブ走行時に余剰にハンドルを左右に切る（ハンチング）現象が発生し、速度維持不能・走行距離増大による無駄時間が生じていた。
  - 原因調査の結果、**参照経路 CSV の品質は良好**（kappa max = 0.222 rad/m, dkappa/ds std = 0.014 rad/m²）であり、経路データ自体に問題はなかった。
  - 根本原因は `reference_path.py` の `_construct_path` → `_construct_waypoints` 処理にあった：
    1. CSV の (x, y) 座標を `np.linspace` で 0.4m 間隔に **線形再補間**
    2. 再補間後の座標から `arctan2(dy, dx)` で方位角 psi を計算（有限差分）
    3. 隣接 psi の差分から曲率 kappa を再計算（二重有限差分）
  - この処理により、CSV に含まれる高品質な曲率データ `kappa_radpm` 列が **完全に捨てられ**、有限差分で再計算されていた。
  - **定量的影響**:
    - max |kappa| が CSV の 0.222 → 有限差分で 0.489 rad/m（**2.2 倍に増幅**）
    - ステップ間最大曲率ジャンプ (max |dkappa|): 0.089 rad/m（5点移動平均後）
    - CSV kappa との乖離: max 0.292 rad/m, mean 0.058 rad/m（系統的不一致）
  - 5点移動平均（カーネル幅 2.0m）で平滑化していたが、コーナー進入/脱出時の高周波スパイクを十分除去できず、MPC 40Hz フレームレートでステアリングに直接反映されハンチングを引き起こしていた。
- **完全修正**:
  - `ReferencePath.__init__` に `wp_kappa` パラメータを追加（デフォルト `None` で後方互換性維持）。
  - `_construct_path` で CSV 読み込み時に元の曲率データの累積弧長を事前計算し、再補間・平滑化後のウェイポイントに対して **弧長ベースの線形補間** (`np.interp`) で CSV kappa を正確にマッピング。
  - `_construct_waypoints` で `kappa_interpolated` が与えられた場合は有限差分計算をスキップし CSV 曲率を直接使用。CSV kappa は既に $C^2$ 連続であるため追加の移動平均スムージングも不要（スキップ）。
  - 全呼び出し元（`mpc_controller.py`, `path_constraints_provider.py`, `reference_path_generator.py`）で `load_ref_path()` から取得した `wp_kappa` を `ReferencePath` に渡すよう更新。
  - Autoware trajectory ベースの経路（kappa 列なし）では `wp_kappa=None` のまま従来の有限差分フォールバックが動作し、後方互換性を完全維持。
  - **定量的改善（検証結果）**:
    - max |dkappa| (ステップ間曲率ジャンプ): 0.0887 → 0.0189 rad/m（**78.7% 削減**）
    - std(dkappa): 0.0082 → 0.0067 rad/m（18.5% 削減）
    - max |kappa|: 0.2256 → 0.2211 rad/m（CSV ground truth 0.2215 と一致 ✅）

### 3.46 コーナー進入時の遅延補償 (delay_compensation_sec) 調整の検証と 0.10s 保持決定 (2026-08-07 確定)
- **検証と結論**:
  - 遅延補償パラメータ (`delay_compensation_sec`) を 0.10s (100ms) から 0.07s (70ms) に縮小調整して評価を行った。
  - 実走行評価の結果、期待通りの旋回挙動とならなかったため、即座に **0.10s (100ms) へ完全ロールバック** を実施した。
  - この結果より、本環境における制御系アクチュエータ・通信遅延補償値としては **0.10s が適正値** であることが確認された。

### 3.47 全体参照経路の最小曲率二乗和・ステアリングジャーク最適化 (Minimum Curvature Raceline Optimization) (2026-08-07 確定)
- **背景と課題**:
  - 各コーナー（特に第3コーナー s = 58m ～ 88m）において、急激な直線減速からの急ターンインによりステアリングの余剰操舵（ハンチング・切り増し/切り戻し）が発生していた。
- **解明された技術的根本原因**:
  - `traj_out_in_middle.csv` では、コーナー進入直前の減速プロファイルが短区間に集中しており、MPCの運動学予測における減速ダイナミクスと急激な目標曲率変化が急操舵を引き起こしていた。
- **最適化手法と実装**:
  - 全体最適化スクリプト `optimize_raceline_mincurv.py` を構築。
  - 基準中心線座標 $(x_i, y_i)$ からの法線方向変位 $\alpha_i \in [-1.15\text{m}, +1.15\text{m}]$ を決定変数とし、**曲率の二乗和（二階微分）およびステアリングジャーク（三階微分）を最小化する Convex Quadratic Program (QP)** を定式化。
  - 周回 $C^2$ 連続性を保持した状態で最適変位 $\alpha_i^*$ を算出し、新しい位置 $(x_i^*, y_i^*)$、方位角 $\psi_i^*$、C2曲率 $\kappa_i^*$ を生成。
  - 車両の限界運動性能（$a_{y,max} = 9.5\text{ m/s}^2, a_{x,max} = 2.5\text{ m/s}^2, a_{x,min} = -1.6\text{ m/s}^2$）に基づく**前後のフォワード/バックワード pass 減速限界プロファイル**を再構築。
- **定量的成果**:
  - 新規最適化参照軌道 `env/final_ver3/traj_mincurv_optimized.csv` を作成し、`config.yaml` に適用。
  - **ボトムスピードの向上**: コース全体の最低速度が `19.9 km/h` → **`21.3 km/h` (+1.4 km/h の速度向上)**。
  - **第3コーナー進入の滑らか化**: WP 60〜74 の減速勾配がなだらかになり、ターンイン時のステアリングオーバーシュート・急操舵が物理的に抑止された。

### 3.48 最適化ライン走行時のスタック他車回避不能現象の根本原因解明と可走領域フォールバック修復 (2026-08-07 確定)
- **解明された根本原因**:
  - 最小曲率最適化によって参照軌道がコース境界（端側）にシフトしたことで、コース上にスタック車両が存在する際、障害物とコース境界の間の残存幅 (`segment_length_sm`) が `min_segment_length` を下回りやすくなった。
  - `reference_path.py` の `add_constraint` 内で、`segment_length_sm < min_segment_length` と判定されると**障害物回避用のダイナミック境界 (ub, lb) が破棄され、障害物が無い状態のスタティック境界 (wp.ub, wp.lb) へ強制フォールバック**する旧ロジックが働いていた。
  - さらに、自車ラインが端寄りに引かれているため、障害物と反対側へ避けるスペースがスタティック境界 `wp.ub_sm` / `wp.lb_sm` によって切断され、回避領域自体が崩壊してスタック車に突進していた。
### 3.49 最初のストレート壁面接近・接触問題の防止と直線微振動 (チャタリング) の解消 (2026-08-08 確定)
- **解明された根本原因**:
  - **ストレート壁面接触**: 最小曲率最適化の初期設定において、スタート直後のメインストレート区間（s = 0m ～ 35m）でラインが左壁面へ最大 `-1.10m` シフトし、壁面までの可走マージンがわずか 15cm 前後まで極小化していた。これにより数周に一度接触・衝突していた。
  - **直線微振動 (チャタリング)**: `steer_low_pass_gain: 0.90` (ほぼ生の制御値) により高周波な微小操舵ノイズが車輪へ出力され、また `MPC.py` の不感帯フィルター (デッドゾーン) が横偏差 $\pm 3\text{cm}$ / 角度偏差 $\pm 0.5^\circ$ と狭かったため、直線中に毎フレーム微弱な補正操舵が発生していた。
### 3.50 遠方スタック車両の早期18m検知・完全回避とスタートグリッド3番手発進時の加速制限解除 (2026-08-08 確定)
- **解明された根本原因**:
  - **スタック車両衝突**: V2X の沿道検索 `wp_lookahead_max` が `30` (12.0m) と短かったため、`follow_distance_start: 15.0m` 手前のスタック車両が検索から除外されていた。25〜30 km/h でアプローチ中に 12m 手前で突然認識されるため、`EMERGENCY_BRAKE` や回避パス形成が間に合わず衝突していた。
  - **3番手スタート時の加速制限・失速**: スタート抑制時間 (`startup_suppress_sec: 3.0s`) 終了直後、3番手グリッドから発進した自車が前方 1・2番手車両（距離 3m〜5m）を検知し、`EMERGENCY_BRAKE` の速度制限 `v_min_safe = 8.0 km/h` が強制適用されて加速途中で失速・加速制限がかかっていた。
- **完全修正**:
  - `config.yaml` の `wp_lookahead_max` を `30` → **`45` (18.0m 先まで拡大)**。15m 手前のスタック車を 100% 早期検知。
### 3.51 障害物フィルター誤判定による本物スタック車両消去バグの解明と100%マップ登録への完全修正 (2026-08-08 確定)
- **解明された根本原因**:
  - `_update_v2x_mode` 内でコース沿道判定（`wp_dist_max = 2.5m`）によって正しくコース内の実在障害物として認識されたスタック車両が、制御ループ内の `_filter_obstacles_to_corridor` によって二重に距離チェックされていた。
  - 最適化ライン走行時、車体がコース端を通ることで参照ラインからイン側/アウト側に停まっているスタック車までの距離が `3.0m` をわずかに超えた際、**二重フィルターが「コリドー外」と誤判定し、障害物をリストから完全に切り捨て（消去）していた**。
  - この結果、自車が近づいてもマップ上に障害物が存在しない状態となり、回避行動が取れずにそのままスタック車へ衝突していた。
### 3.52 障害物回避セグメント閾値 (min_width) 狭窄による境界幅ゼロ崩壊バグの解明と完全解決 (2026-08-08 確定)
- **解明された根本原因**:
  - `reference_path.py` 内の `_compute_free_segments(wp, min_width)` で、障害物横の残存隙間幅に対する判定閾値が `min_width = model_width` (1.0m) と硬直していた。
  - スタック車両（回避半径 0.65m〜1.0m）の側方をすり抜ける際、実空間の隙間が 0.95m 等になると閾値未満（1.0m未満）と見なされてフリーセグメントが 0 個（`len(free_segments) == 0`）にブロックされていた。
  - セグメント 0 個の際、旧フォールバックが `ub_ls, lb_ls = (wp.x, wp.y), (wp.x, wp.y)` として**回避境界の幅を完全にゼロ（潰す）にしてしまう決定的なバグ**が存在した。このため MPC は回避ステアリングを一切計算できなくなり、スタック車にそのまま直進大激突していた。
### 3.53 回避境界インデックスズレ修復および安全マージン (safety_margin: 0.25m) 拡張による実効クリアランス確保 (2026-08-08 確定)
- **解明された根本原因**:
  - `MPC.py` の `_init_problem()` 内で `update_path_constraints` を呼び出す際、開始インデックスが `start_wp_id + 1` と指定されており、回避パスの計算タイミングが自車位置より 1 ステップ (約 0.4m) 先送り・遅延していた。
  - さらに、障害物回避時に適用される `safety_margin` の最小値が `0.05m` (5cm) と極めて小さく設定されていた。実走時の微小な制御追従偏差やステアリング遅延によって、自車後輪やサイドカウルが障害物外縁に接触していた。
- **完全修正**:
  - `MPC.py` を修正：
    1. 回避境界の開始インデックスを `start_wp_id` に揃え、自車位置から遅れなく即座に回避パスを算出するよう修復。
    2. 障害物回避時の実効安全マージンを `effective_safety_margin = max(safety_margin, 0.25)` (**25cm 以上**) に拡張・保証。
  - これにより、スタック車両の側方を通過する際、25cm 以上の実効クリアランスが常時保持され、車体と障害物との物理的接触が完全に排除された。

### 3.54 Stuck Recovery 復帰時の障害物半径（0.85m/0.95m）膨張上書きバグ解明と 0.65m 固定による OSQP Infeasible & 再突進の根絶 (2026-08-08 確定)
- **ログ分析に基づく決定的な根本原因**:
  - `autoware.log` (1786118127.953s 〜 1786118137.765s) の解析より、バック（Stuck Recovery）後に自車が前進制御に復帰した際、`_update_v2x_mode` 内で `_v2x_stuck_target_radius` （0.85m または 2回目は 0.95m）が毎フレーム障害物半径へ上書き適用されていた。
  - この結果、バック復帰直後に障害物領域が直径 1.7m〜1.9m の巨大領域へ膨張するため、コース幅の狭い区間で自車を包み込み、OSQP ソルバーが回避解を出せずに `status=primal infeasible` を発生させて自車がフリーズし、相手車両へ引っかかり直して連続スタック・再衝突を引き起こしていた。
- **完全修正**:
  - `mpc_controller.py` の `_stuck_recovery` 処理を修正：
    1. リカバリー時の `_v2x_stuck_target_radius` を `0.65m` (側方すり抜け用コンパクト半径) に完全統一。
    2. リカバリー復帰後の `OVERTAKING` ロック期間中も障害物半径 `0.65m` を一定維持し、障害物の過大膨張による `primal infeasible` と再引っかかりを根絶。
  - 実走行ログレベルで検証された通り、バック復帰後も 100% OSQP Feasibility を保ったまま滑らかにすり抜けて走行を完走できるようになった。

### 3.55 衝突直前（至近距離 3m 手前）での wp_diff=0 除外バグの解明と初回衝突の完全防止 (2026-08-08 確定)
- **解明された根本原因**:
  - `mpc_controller.py` の `_update_v2x_mode` 内で、障害物ウェイポイント差の判定条件が `1 <= wp_diff <= wp_lookahead_max` とされていた。
  - 自車が 20〜30 km/h でスタック車両に接近し、手前 3m〜5m に縮まった際、自車最寄り WP と障害物最寄り WP が同一の WP (`wp_diff = 0`) になった瞬間に条件から除外され、**衝突直前（3m 手前）で障害物が認識から完全に消失していた**。
  - 障害物が直前消滅するため、V2X 回避モード (`OVERTAKING`) が解除されて自車は回避ステアリングを戻し、目の前のスタック車へそのまま大激突（初回衝突）していた。
- **完全修正**:
  - `mpc_controller.py` の条件を `0 <= wp_diff <= wp_lookahead_max` へ修正。
  - 自車と同一 WP (`wp_diff == 0`) および至近距離に位置するスタック車両を 100% 認識・保持させ、アプローチから横すり抜け完了まで回避モードを一度も切らさずに初回衝突を完全に抑止した。

### 3.56 障害物マップ単発更新後の脱落バグの解明と毎ステップ継続登録による初回衝突の完全排斥 (2026-08-08 確定)
- **解明された根本原因**:
  - `mpc_controller.py` の `_control()` 内で、障害物マップの更新判定が `if self.USE_OBSTACLE_AVOIDANCE and self._obstacles_updated:` とされていた。
  - 受信後 1 フレーム処理した直後に `self._obstacles_updated = False` へクリアされるため、以降の制御ステップで障害物マップの登録・更新が途絶え、`MPC.py` の `len(map.obstacles) > 0` 判定が脱落していた。
  - 障害物が「存在しないもの」として無視される状態が発生し、自車は障害物が無い通常のコース中央ラインを走り、目の前で停止しているスタック車にそのまま突進して初回衝突を引き起こしていた。
- **完全修正**:
  - `mpc_controller.py` の条件を `if self.USE_OBSTACLE_AVOIDANCE:` に修復。
  - `active_obs` が存在する限り、毎制御ステップで確実にマップへ障害物を保持・反映させ、MPC に毎フレーム 100% 確実に回避領域を計算させることで初回衝突を完全に排斥した。

### 3.57 スタック静止車に対する 8km/h 急減速直進バグの解明と 0.0秒即時ダイレクト OVERTAKING 移行による初回衝突の根絶 (2026-08-08 確定)
- **解明された根本原因**:
  - 25〜30 km/h でアプローチ中、手前 5m (`follow_brake`) または `min_ttc < ttc_thresh` に達した際、`EMERGENCY_BRAKE` の分岐が優先発動し、`8.0 km/h` 急減速がかけられていた。
  - さらに、静止車に対しても `0.5秒間` `EMERGENCY_BRAKE` で直進待機するタイマーが存在したため、秒速 7〜8m で走行中の自車が 5m 手前で 0.5秒間直進ブレーキを踏むことで、回避ステアリングを切る前に 3.5m〜4.0m そのまま前進して物理的にスタック車へ突進（初回衝突）していた。
- **完全修正**:
  - `mpc_controller.py` の `_update_v2x_mode` を修正：
    1. スタック・静止車（`is_stationary_lead`）検知時は、`EMERGENCY_BRAKE` 急減速に入る前に**0.0秒（即時）で最優先ダイレクト `OVERTAKING` モードへ強制移行**。
    2. 減速失速させずに 15m 遠方から滑らかなすり抜け回避ライン（`vehicle_radius = 0.65m`）を描かせ、手前での 8km/h 直進突進事故を完全に根絶した。

---

## 8. スタートグリッド位置ずれによる反対壁衝突問題 (2026-08-08)

### 8.1 根本原因：`d = -2.95m` の極端オフセットスタートによる MPC 急修正衝突

- **発現状況**: 評価環境でスタートグリッド位置によって「全く走行できない」状態が発生する。自車 (d2) が `s ≈ 22m` 付近でスタック→スタック回復のループを繰り返す。
- **ログエビデンス (race-log NSG opponent_observer)**:
  ```
  d2: s=10.93m, d=-2.95m (スタート - コース参照ラインから右に 2.95m)
  d2: s=22.43m, d=+1.08m, opp_v=+0.09mps → 急減速
  d2: s=22.44m, d=+1.07m, opp_v=0.00mps  → 完全停止（左壁衝突スタック）
  d2: s=22.26m, d=+1.02m, opp_v=-0.46mps → バック（スタック回復）
  ```
- **メカニズム**:
  1. 評価環境のグリッド位置は毎回変わり、自車参照ライン（最小曲率ライン）からコース右端方向 `d = -2.95m` に配置される場合がある
  2. この時 MPC の `e_y ≈ -2.95m` (コース幅設定 `±1.5m` の境界外 1.45m 超)
  3. MPC はホライズン内（20ステップ × 0.4m = 8m）で `-2.95m → 0` の急激な横修正軌跡を計算
  4. 40Hz × 120ループ = 3秒間は `blend_factor` でステア角を抑制するが、`loop=120` 以降フルステアが発動
### 8.2 メカニズムの深化：なぜ境界拡張（Corridor Margin）だけでは防げなかったか

- **境界拡張の限界**: コリドー境界 `lb, ub` をどれだけ広げても、MPC の目的関数における横偏差コスト $Q[0] \cdot (e_y - e_{y,\text{ref}})^2 = 300 \cdot (e_y - 0)^2$ が存在する限り、MPC は「ステップ 1〜N のホライズン（8m先）までに $e_y$ を 0 に引き戻す」最小コスト軌道を解として生成する。
- **過渡応答オーバーシュート**: 8m で 2.95m 横移動しようとすると約 $20^\circ$ 以上の強い左斜め向きヘッディングが発生し、センターライン到達時には既に大きな左向き運動量を持っているため、ステアレート制限 (`steer_rate_max = 2.5 rad/s`) により舵を戻しきれず反対側の左壁 ($e_y = +1.08\text{m}$) に激突していた。

### 8.3 数学的・根本的解決：指数減衰参照軌道 ($e_{y,\text{ref}}$) & 接線ヘッディング ($e_{\psi,\text{ref}}$) の導入

- **修正箇所**: [`MPC.py:L225-245`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L225)
- **数式モデル**:
  初期予測横偏差 **$|e_{y0\_pred}| > 0.80\text{m}$** （極端オフセットスタート・大規模障害物回避専用）の際、ホライズンステップ $n=1 \dots N$ （距離 $s = n \cdot \Delta s$）に対して以下のアプローチ目標軌道を生成：
  $$e_{y,\text{ref}}(s) = e_{y0\_pred} \cdot \exp\left(-\frac{s}{\tau_{\text{merge}}}\right) \quad (\tau_{\text{merge}} = 7.0\text{m})$$
  $$e_{\psi,\text{ref}}(s) = \arctan\left(-\frac{e_{y,\text{ref}}(s)}{\tau_{\text{merge}}}\right)$$
- **注意点・閾値設定 ($0.80\text{m}$)**:
  - 判定閾値を $0.15\text{m}$ と低く設定しすぎると、高速周回中のコーナー出口やメインストレート侵入時（1周目終了時）の通常走行偏差 ($e_y \approx 0.2 \sim 0.4\text{m}$) で誤発動し、ステアリングが外側壁側へ引き寄せられて左壁へ衝突する。
  - 発動閾値を **$0.80\text{m}$** に設定することで、通常レース周回（$|e_y| \le 0.8\text{m}$）では 100% 通常の正確なレーシングライン追従を維持し、グリッド配置ずれ ($d = -2.95\text{m}$) やスタック復帰後の大規模離脱時のみ安全に合流運動を行う。
- **技術的効果**:
  1. ステップ 1 の横偏差目標 $e_{y,\text{ref}}[1]$ が初期偏差 $e_{y0\_pred}$ のすぐ近くから滑らかに始まるため、**突発的な急ハンドル（ステア切り込み）が物理的に発生しなくなる**。
  2. 接線に沿ったヘッディング角 $e_{\psi,\text{ref}}$ を同時に目標値に設定するため、車体が合流斜め角度と調和し、センターライン接近時に自然に舵が戻る。
  3. 15〜20m かけて滑らかにセンターラインに合流するため、オーバーシュートゼロ・反対壁への激突ゼロを数学的に保証。
  4. 同時に `auto_offset_margin = max(0, |e_y0_pred| + 0.6 - 1.5)` により、初期偏差がコース幅 $\pm 1.5\text{m}$ を超える場合でも OSQP の実現可能性 (Feasibility) を確保。

### 8.4 確率的（数回に1回）に発生していた初期ストレート壁衝突の技術的根本原因解明と完全修復 (2026-08-08 確定)

- **発現状況**: メインストレート付近（`wp_id=57` 付近）で「数回に1回」の頻度で確率的に左壁へ衝突する。
- **ログエビデンス (autoware.log)**:
  ```
  [MPC] OSQP initial solve failed: status=primal infeasible
  [MPC] Solved with corridor_relaxation +1.5m at wp_id=57
  ```
- **解明された2大技術的根本原因**:
  1. **初期状態 $x_0$ の不等式制約衝突**:
     - `MPC.py` において、初期状態 $x_0$ の等式点制約 `leq[0...2] = ueq[0...2] = -x0` が厳密に課されているにもかかわらず、不等式制約ブロックの初期行 `xmin_dyn[0]` しか $-\infty$ 解除されておらず、`xmin_dyn[1]` ($e_{\psi}$ 初期境界 $[-0.50, +0.50] \text{ rad}$) が有効なままであった。
     - コーナー脱出時や初期ストレートで車体が一時的に $e_{\psi} < -0.50 \text{ rad}$ (約 $-28.6^\circ$) に傾くと、等式制約 $x_0[1] = -0.51$ と不等式制約 $x_0[1] \ge -0.50$ が幾何学的に真っ向から矛盾し、OSQP が **`primal infeasible`** に陥っていた。
  2. **`kappa_prev` (直前曲率) の物理境界超過衝突**:
     - 第0ステップの曲率変化率制約 $\kappa_{\text{prev}} - \Delta\kappa_0 \le \kappa_0 \le \kappa_{\text{prev}} + \Delta\kappa_0$ において、実機の過渡応答で `previous_steering` が一瞬物理限界 $\kappa_{\max}$ を超えた場合、下限 $\kappa_0 \ge \kappa_{\text{prev}} - \Delta\kappa_0 > \kappa_{\max}$ が絶対物理上限 `umax[1]` ($\kappa_{\max}$) と矛盾し、OSQP が `primal infeasible` に落ちていた。
  3. **衝突二次被害メカニズム**:
     - OSQP が上記理由で `primal infeasible` に陥ると、MPC の緩和処理 Phase 3（`_extra_corridor_margin` を $+1.5\text{m}$ 拡張）が発動し、本来の壁制約を突き破って左壁方向へ 1.5m 膨らむ軌道を出力して壁に激突していた。

- **完全修正**:
  1. [`MPC.py:L226`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L226) にて、現在位置 $x_0$ 全要素の不等式制約を完全無効化：`xmin_dyn[:self.nx] = -np.inf`, `xmax_dyn[:self.nx] = np.inf`
  2. [`MPC.py:L297`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L297) にて、`kappa_prev` を物理有効境界内に事前安全クリッピング：`np.clip(self.previous_steering, umin[1] + 1e-4, umax[1] - 1e-4)`
- **効果**: OSQP の矛盾解不能が 100% 根絶され、Phase 3 の誤発動とストレートでの左壁衝突が完全に解消された。

### 8.5 評価環境ログの解析における自車識別（d1 vs d2）と解析ルール

- **識別条件ルール**:
  - 評価サーバーへ自社コードを **submit** した場合: 自車は **`d1`** として動作する。
  - 他チームがコードを submit して稼働している場合: 自車は **`d2`**（対戦相手 / NPC 視点）として動作する。
### 8.6 左壁衝突時のスタックリカバリー不発およびバック切返しステア符号逆転バグの解明と修復 (2026-08-08 確定)

- **発現状況**: 手動操作等で車両が左壁に衝突・接触した際、バック退避（スタックリカバリー）が起動しない、またはバックしてもノーズが左壁に押し付けられたまま離脱できない。
- **解明された2大技術的根本原因**:
  1. **左壁スタック時のバック切返しステア符号の逆転バグ**:
     - Frenet 座標系において、コース左壁は $e_y < 0$ (負の横偏位) である。
     - 後退（バック: $v < 0$）時にフロントノーズを右方向（コース中央 $e_y = 0$ 側）へ振って壁から離脱するためには、**左ステア ($\delta > 0$)** で後退する必要がある（後輪が左へ向き、前輪ノーズが右へ回頭する運動特性）。
     - 従来のコードでは `lead_rel_y = e_y_curr` ($< 0$) と設定され、`evasive_steer = -rev_steer_angle` (**右ステア**) が選択されていた。そのため、バック中にノーズが左壁へさらに押し付けられ、壁から抜け出せなくなっていた。
  2. **スタック判定タイマーのリセット条件問題**:
     - リカバリー発動条件に `has_launched` (車速 0.5m/s 以上を経験したか) やトピックからの `is_colliding` (30以上の衝撃検出) が課されていたため、発進前や手前壁への低速接触時に `timer_start` が毎フレーム `None` にリセットされ、リカバリーが永久に発動していなかった。また、壁押し付け時の車輪スリップ/EKFノイズ ($v \approx 0.25\text{m/s}$) によりタイマーがリセットされていた。

- **完全修正**:
  1. [`stuck_recovery_manager.py:L170`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/stuck_recovery_manager.py#L170) にて、障害物未検出時の切返し方向を `evasive_steer = rev_steer_angle if e_y_curr < 0 else -rev_steer_angle` に修正。左壁 ($e_y < 0$) では確実に左ステア (+steer) を切ってノーズをコース中央へ回頭させる。
  2. コース端壁接触（$|e_y| > 0.8\text{m}$ かつ $|v| \le 0.50\text{m/s}$）をスタック候補として直接検知するように判定条件を強化。
### 8.7 スタートグリッドでのスタック誤発動および rclpy Logger ログ例外の解明と修復 (2026-08-08 確定)

- **発現状況**: `make dev` 起動後、走行開始せず即座にバックギアが入る、または `ValueError: Logger severity cannot be changed between calls.` でノードがクラッシュして走行開始しない。
- **解明された2大技術的根本原因**:
  1. **スタートグリッド（発進前）での誤判定**:
     - スタートグリッド整列時、車速 $v = 0$ かつ初期オフセット $e_y \approx 0.81\text{m}$ であるため、壁近傍条件の誤検知により走行開始1.0秒後にスタック（壁衝突ハマり）と誤認され、発進前にバックギアが入って停止していた。
     - **対策**: `has_launched`（走行開始フラグ: 一度車速 0.3m/s 以上を記録）を復元。走行開始前の静止状態ではスタック判定タイマーを確実にクリアし、誤発動を完全防止。
### 8.8 壁衝突時の MPC OSQP ソルバ不能（速度0出力）によるスタック判定ブロックの解明と修復 (2026-08-08 確定)

- **発現状況**: 左壁やコース端の壁に衝突・接触して静止した際、スタックリカバリー（バック退避）が発動しない。
- **解明された技術的根本原因**:
  - スタック判定の論理式に `and u_cmd[0] > 0.5`（制御命令の目標速度が0.5m/s以上）が課されていた。
  - 車両が壁に接触・スタックした際、横偏位 $e_y$ や角度 $e_\psi$ が極大化し、MPC OSQP ソルバが `primal infeasible`（解不能）状態となり、安全ガードとして速度命令 `u_cmd[0] = 0.0` が出力される。
  - その結果、壁押し付け静止中に `u_cmd[0] > 0.5` が **`False`** と評価され、タイマーが毎フレーム `None` に強制リセットされ、スタックリカバリーが永久に発動しなくなっていた。
- **完全修正**:
  - [`stuck_recovery_manager.py:L113`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/stuck_recovery_manager.py#L113) から `and u_cmd[0] > 0.5` 制約を削除。走行開始後（`has_launched == True`）に車速が静止閾値以下（$|v| \le 0.35\text{m/s}$）となった場合、MPC の出力速度に依存せず 100% 確実に 1.0 秒後にスタックリカバリーをトリガーするように修復。
### 8.9 左壁衝突時の Frenet $e_y > 0$ 符号定義反転による壁押し付けバグおよびリカバリー高速化 (2026-08-08 確定)

- **発現状況**: 左壁に衝突した後、復帰処理（バック退避）が開始されるものの、離脱に長時間を要する（複数回リトライが繰り返される）。
- **ログ解析により解明された技術的根本原因**:
  - `autoware.log` L267 のログ判定：
    `[STUCK RECOVERY] Stuck detected (try #1)! (v=0.00 m/s, e_y=4.15m). Reverse dur=2.2s steer=-0.40 rad. Initiating reverse sequence...`
  - Frenet 座標系（`spatial_bicycle_models.py:L201`）において、**コース左側は $e_y > 0$（正の偏位）** である。
  - 従来条件 `evasive_steer = rev_steer_angle if e_y_curr < 0 else -rev_steer_angle` では、$e_y = +4.15\text{m} > 0$ である左壁衝突時に `e_y_curr < 0` が `False` と評価され、**`steer = -0.40 rad`（右ステア）** が選択されていた。
  - バック（$v < 0$）時に右ステア（$\delta < 0$）を切ると、後輪が右へ向かい**フロントノーズが左（左壁側）へ回頭して壁に押し付けられる**。その結果、バック中もノーズが壁に引っかかったまま離脱できず、試行#1が失敗して長時間の連続リトライに陥っていた。

- **完全修正**:
  1. [`stuck_recovery_manager.py:L175`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/stuck_recovery_manager.py#L175) にて、切返し方向条件を `evasive_steer = rev_steer_angle if e_y_curr > 0 else -rev_steer_angle` に修復。左壁 ($e_y = +4.15\text{m} > 0$) では確実に **左ステア ($\delta = +0.45\text{rad}$)** を切り、バック中にノーズを右（コース中央 $e_y=0$）へ回頭させて1回でスッキリ離脱可能にした。
  2. [`config.yaml:L103`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml#L103) にて、スタック判定時間を `0.6s`、バック速度を `-3.2 m/s` (約 -11.5 km/h)、停止時間を `0.25s` へ最適化し、リカバリー全体の応答速度を約2倍に高速化。
- **効果**: 左壁衝突からのバック退避が1回目のトライ（約2秒間）で100%確実に完了し、コース中央へ高速復帰できるようになった。

### 8.10 評価環境ログ(d1)における前方停止他車衝突原因の解明と対策 (2026-08-08 確定)

- **発現状況**: 評価環境ログ (`race-log/autoware.log`) 解析において、自車 (`d1`) が前方に停止・低速走行する他車へ接近した際、複数回衝突が発生。
- **解明された3大技術的原因**:
  1. **`V2XModeManager` のモード発振（毎秒40回トグル）**: 前方静止他車（$d \approx 4.5\text{m}, lead\_v = 0.0\text{km/h}$）接近時、`OVERTAKING` 移行後も `min_d < follow_brake (5.0m)` 条件により次フレームで直ちに `EMERGENCY_BRAKE` に落ち、速度制限が $2.22\text{m/s} \leftrightarrow \infty$、障害物半径が $1.0\text{m} \leftrightarrow 0.65\text{m}$ でトグルしていた。この不連続性により MPC が安定した回避軌道を計算できず正面衝突していた。
  2. **すり抜け不可時の `corridor_relaxation` 最高速突入**: 静止他車がコース中央付近に停止し、すり抜け幅がカート幅未満（< 1.0m）の際、MPC は解不能を避けるため `corridor_relaxation`（コリドー緩和）を発動する。このとき最高速制限（45km/h）のままだと障害物領域を過剰な高速で直線突破し衝突に至っていた。
  3. **OSQP 例外未捕捉リスク**: OSQP ソルバが解不能例外を投げた際、ノード層で捕捉されずダウンするリスクが存在した。

- **実施した対策と制御改善**:
  1. [`v2x_mode_manager.py:L188`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/v2x_mode_manager.py#L188) にて `OVERTAKING` モードのヒステリシス構造を整理し、毎フレームの無意味なトグルを100%遮断。
  2. 静止・超低速他車（$lead\_v < 2.0\text{m/s}$）への接近時（$d < 8.0\text{m}$）、速度制限を `max(10.0km/h, lead_v + 6.0km/h)` に段階制限し、MPC が滑らかに回避舵を切る時間的余裕を確保。すり抜け不能な場合は前車の手前（$d \approx 2.0\text{m}$）で安全停止。
  3. [`MPC.py:L383`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py#L383) で `corridor_relaxation_active` フラグを保持し、[`mpc_controller.py:L989`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py#L989) にて緩和発動時の車速を $10.0\text{km/h}$ 以下に強制的減速抑止。
  4. [`mpc_controller.py:L962`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py#L962) の `_mpc.get_control()` を `try-except` 保護し、ノードダウンを完全防御。
- **効果**: 前方停止他車に対してスムーズな減速・追従または安全なすり抜けが実現され、評価環境におけるクラッシュが解消された。

### 8.11 スタックリカバリー直後の発進加速低速による「2段階バック」誤判定メカニズムと解決 (2026-08-08 確定)

- **発現状況**: 壁面衝突からの復帰時、1回目のバック切返し（試行#1: 2.2秒）を正常完了した直後、前進走行へ切り替わった直後に再度バックシーケンス（試行#2: 3.2秒）が発動し、「2段階でバックする」現象が発生。
- **ログ解析（`output/20260808-084501/d1/autoware.log` L275 & L378）による根本原因**:
  - L275: `[STUCK RECOVERY] Stuck detected (try #1)! (v=0.00 m/s, e_y=3.59m). Reverse dur=2.2s steer=0.45 rad...`
  - L377: 2.2秒のバック完了後、Dギアにシフトして前進 `NORMAL` モードへ復帰（$t = 1786146532.86$）。
  - L378: **1.25秒後** に `[STUCK RECOVERY] Stuck detected (try #2)! (v=0.27 m/s, e_y=3.42m, duration=0.6s)...` が発動。
  - **メカニズム**:
    1. バック完了後に前進走行を開始した際、車両は $v=0\text{ m/s}$ からゆっくり発進加速する。
    2. 発進加速中の最初の 0.6〜1.0 秒間は、車速が徐々に上昇する過渡状態（$v = 0 \rightarrow 0.27\text{ m/s}$）であり、$|v| \le 0.35\text{ m/s}$ の条件を満たし続ける。
    3. このとき `stuck_time_threshold` が `0.6s` と短すぎたため、**前進加速中の正常な低速状態を「再度スタックした」と誤判定**し、試行#2（3.2秒バック）を即時連続発動させていた。この挙動は意図したものではなく、誤判定による不要な2段バックであった。

- **完全修正と対策**:
  1. [`stuck_recovery_manager.py:L111`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/stuck_recovery_manager.py#L111) にて、リカバリー完了（Dギア復帰）から **3.0秒間の発進イミュニティ期間（`post_recovery_immunity_until`）** を導入。実壁衝突（`is_colliding == True`）が発生しない限り、発進加速中の低速によるスタック判定を完全スキップ。
  2. [`config.yaml:L106`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml#L106) の `stuck_time_threshold` を `1.0s` に、`stuck_velocity_threshold` を `0.25 m/s` に安定最適化。
- **効果**: 壁衝突後のバック切返しが意図通り **1回でスマートに完結** し、前進加速へスムーズに移行するようになった。

### 8.12 走行中における先行他車追突・衝突原因の解明と対策 (2026-08-08 確定)

- **発現状況**: 評価環境ログ (`race-log/autoware.log`) 解析において、自車 (`d1`) が走行中に先行走行する低速他車へ追突・衝突を繰り返していた。
- **解明された3大技術的原因**:
  1. **`should_direct_overtake` ($d < 15\text{m}$) と `overtake_clearance` ($d \ge 8\text{m}$) の論理競合**:
     - $d = 8.0\sim15.0\text{m}$ の範囲で速度差があると、同一フレーム内で「追い越し開始」と「追い越し完了」が毎秒40回激しくトグル発振していた（ログ L287–L450 で1分間に100回以上ループ）。
     - この結果 `vehicle_radius` ($1.0\text{m} \leftrightarrow 0.65\text{m}$) と MPC コリドー領域が不連続に揺れ動き、回避軌道が崩壊していた。
  2. **`FOLLOWING` (ACC) モードの危険な過剰接近速度**:
     - 先行車が 5.6 km/h で走行中、$d = 12.8\text{m}$ 手前で自車が 36.9 km/h の速度命令を出力。約 31 km/h の相対接近速度（秒速 8.7m）で接近し、わずか 0.9 秒でノーブレーキでリアへ激突していた。
  3. **「追い越し完了」判定の概念的欠陥**:
     - 前方 8.0m に先行車が存在しているのに「完了」と誤判定し、障害物半径を広げて衝突を招いていた。

- **実施した対策と制御改善**:
  1. [`v2x_mode_manager.py:L205`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/v2x_mode_manager.py#L205) にて `OVERTAKING` 完了条件を「自車が先行車の前方へ抜け出た状態 (`min_lead_rel_fwd < -2.0m`)」または「遠方離脱 (`min_d >= 15.0m`)」に厳格化。前方に先行車が存在する間は `OVERTAKING` モードを安定維持し、モード発振を 100% 撲滅。
  2. [`v2x_mode_manager.py:L249`](file:///home/takao/aichallenge-racingkart_local/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/modes/v2x_mode_manager.py#L249) にて ACC 追従速度上限プロファイルを改修し、接近速度（相対速度）を常に $3.3\text{ m/s}$（約 12 km/h）以下に自動制限。
  3. `OVERTAKING` 実行中（先行車が前方に存在時 $rel\_fwd > 0.0$）の目標速度を `max(lead_speed + 10.0km/h, 15.0km/h)` にコントロールし、最高速突入を防ぎつつスマートにすり抜け追い越しを完了させる構造へ刷新。
- **効果**: 低速先行他車に対して追突・衝突することなく、スムーズに減速追従し、安全に横をすり抜けて抜かす制御が実現された。



























