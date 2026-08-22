# 自律走行レーシング制御 改善総括サマリー

自律走行レーシングカー（d1）の競技走行における、複数台走行環境（d2, d3）での安定したオーバーテイク、接触防止、壁面衝突からの即時復帰を実現するための一連の改善サマリーです。

詳細なアーキテクチャ設計書は [ADR-009](file:///home/takao/NICS_Repo_naito/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros_with_dynamic_param/doc/adr_009_overtake_recovery_and_dynamic_racing_improvements.md) をご参照ください。

---

## 🎯 改善項目サマリー

| 項目 | 課題 | 実施した対策 | 効果 |
| :--- | :--- | :--- | :--- |
| **① RecoveryState 一発復帰** | 接触時にリバース操舵角が逆で壁に潜り込み無限バック | `steer_cmd = +1.2 * psi_err` の正味補正 ＆ 復帰離脱時にステアリング中立化（`_last_u[1] = 0.0`） | わずか 1.5〜1.8 秒・1 回のリバースで確実にコース中央へ前進復帰 |
| **② FollowState ジェントル追従** | 先行車に追いついた瞬間に急減速しモメンタム喪失 | 相対速度連動の比例追従 ＆ 下限フロア速度（先行車速 $- 1.0\text{m/s}$）ガード | 急失速を防ぎ、高い車速を保ったまま追い越しチャンスを維持 |
| **③ 実測オープン空間選択** | コーナー曲率判定のみでインの狭い壁側へ突入し衝突 | LiDAR/V2X による左右実測空間比較（100% 広い側を選択）＆ 壁マージン $\ge 1.35\text{m}$ 死守 | 先行車のライン取りに合わせて空いている側（左 5.8m / 右 3.3m）へ的確に飛び出し |
| **④ 並走フル加速ロック** | 並走中に相手が正面から外れて誤合流、10km/h 付近へ失速 | V2X 相対座標（$-4\text{m} \le x_{\text{rel}} \le 4\text{m}$）監視、加速度 $3.5\text{m/s}^2$ ＆ 目標車速 $38\text{km/h}$ 解放 | 10km/h 失速を根絶し、相手の横に並んだ瞬間に一瞬で抜き去る |
| **⑤ 合流マージン拡大** | 追い抜き完了直後に相手のノーズに接触・コーナー前交差 | 前後安全マージン $x_{\text{rel}} < -4.5\text{m}$、コーナー前ブレーキングゾーンでの合流禁止、タイムアウト 6.0 秒延長 | 追い抜き後の合流時および第1コーナー進入時の接触をゼロ化 |
| **⑥ 3段階先行車速度判定** | 壁から復帰中の低速車の後ろで 23 秒間待機スタック | ①完全停止（$<1\text{m/s}$）、②低速復帰中（$<5\text{m/s}$）、③通常走行車の 3 階層判定 | 復帰中の遅い車両の真後ろで止まることなくスマートにバイパス |
| **⑦ ヘアピン追い越し解禁** | 超急ヘアピンで一律禁止のためガラ空きでも抜けなかった | ワイドスペース（$\ge 2.60\text{m}$）時にヘアピン追い越し解禁 ＆ ヘアピン旋回速度制御（$24\text{km/h}$） | スペースさえあれば超急ヘアピンであっても安全に小回りしてパス |

---

## 📦 関連ファイル
* 設計文書: [`adr_009_overtake_recovery_and_dynamic_racing_improvements.md`](file:///home/takao/NICS_Repo_naito/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros_with_dynamic_param/doc/adr_009_overtake_recovery_and_dynamic_racing_improvements.md)
* 状態遷移制御: [`multi_purpose_mpc_ros_with_dynamic_param/states.py`](file:///home/takao/NICS_Repo_naito/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros_with_dynamic_param/multi_purpose_mpc_ros_with_dynamic_param/states.py)
* 速度・加速度制御: [`multi_purpose_mpc_ros_with_dynamic_param/mpc_controller.py`](file:///home/takao/NICS_Repo_naito/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros_with_dynamic_param/multi_purpose_mpc_ros_with_dynamic_param/mpc_controller.py)
