# ADR-027: Waypoint 沿い道のり（累積アーク長差）による前方車両相対距離算出

## ステータス
承認済み (Accepted)

## コンテキスト
従来のユークリッド直線距離（$\sqrt{\Delta x^2 + \Delta y^2}$）や車体ローカル縦座標 $x_{\text{rel}}$ による前方車両距離判定では、ヘアピンカーブやS字コーナーにおいてコース外の直線近傍車を誤検知したり、カーブに沿った実際の走行道のり距離との乖離が生じていた。
これを解決するため、自車最寄り Waypoint と前方車両最寄り Waypoint 間の「コース進行方向に沿った累積アーク長（道のり）の差」を用いて相対距離（車間距離）を算出する方式を導入する。

## 決定事項

### 1. 道のり距離（アーク長差）の算出アルゴリズム
1. **累積アーク長配列 $S$ とトラック全長 $L$ の事前計算**:
   - Waypoint 配列から各セグメント距離 $\Delta s_k$ を累積し、$S = [s_0, s_1, \dots, s_{N-1}]$ および $L = \sum \Delta s_k$ を構築。
2. **最寄り Waypoint の検索**:
   - 自車位置 $(x_{\text{ego}}, y_{\text{ego}}) \implies i_{\text{ego}} = \arg\min_k \| \mathbf{p}_k - \mathbf{p}_{\text{ego}} \|^2$
   - 対象車位置 $(x_v, y_v) \implies i_v = \arg\min_k \| \mathbf{p}_k - \mathbf{p}_v \|^2$
3. **周回サーキット対応の前進道のり距離 $s_{\text{rel}}$**:
   $$d_{\text{path\_fwd}} = (S[i_v] - S[i_{\text{ego}}]) \pmod L$$
   $$s_{\text{rel}} = \begin{cases} d_{\text{path\_fwd}} & \text{if } d_{\text{path\_fwd}} \le L / 2.0 \\ d_{\text{path\_fwd}} - L & \text{if } d_{\text{path\_fwd}} > L / 2.0 \end{cases}$$
   - $s_{\text{rel}} > 0$: 自車より前方の道のり距離 [m]
   - $s_{\text{rel}} < 0$: 自車より後方の道のり距離 [m]

### 2. 適用箇所
- `mpc_controller.py`:
  - `_detect_forward_and_side_vehicles`: 前方車両の選定および `forward_vehicle_distance` への設定に $s_{\text{rel}}$ を適用。
  - `_scan_surrounding_vehicles`: 前方車両判定（$0 < s_{\text{rel}} \le \text{fwd\_detect\_distance}$）および距離ソートに $s_{\text{rel}}$ を適用。

## 影響と効果
- ヘアピンや急カーブ走行時でも、コースの曲がりに即した真の車間距離（道のり）が正確に把握され、追従速度 PID 制御や追い越し状態遷移判定の精度と安定性が飛躍的に向上する。
