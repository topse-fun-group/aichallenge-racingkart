from typing import Tuple
import numpy as np
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

# Colors
PREDICTION = '#BA4A00'

##################
# MPC Controller #
##################

class MPC:
    def __init__(self, model, N, Q, R, QN, StateConstraints, InputConstraints,
                 ay_max, max_steering_rate, wp_id_offset, use_obstacle_avoidance, use_path_constraints_topic, use_max_kappa_pred=True):
        """
        Constructor for the Model Predictive Controller.
        :param model: bicycle model object to be controlled
        :param N: time horizon | int
        :param Q: state cost matrix
        :param R: input cost matrix
        :param QN: final state cost matrix
        :param StateConstraints: dictionary of state constraints
        :param InputConstraints: dictionary of input constraints
        :param ay_max: maximum allowed lateral acceleration in curves
        :param wp_id_offset: offset for waypoint id to consider control delay
        :param use_obstacle_avoidance: flag to enable obstacle avoidance
        :param use_path_constraints_topic: flag to use path constraints from topic
        :param max_steering_rate: maximum allowed steering rate in rad/s
        """
        # 既存の初期化パラメータ
        self.N = N
        self.Q = Q
        self.R = R
        self.QN = QN
        self.wp_id_offset = wp_id_offset
        self.use_obstacle_avoidance = use_obstacle_avoidance
        self.use_path_constraints_topic = use_path_constraints_topic
        self.model = model
        self.nx = self.model.n_states
        self.nu = 2
        self.state_constraints = StateConstraints
        self.input_constraints = InputConstraints
        self.ay_max = ay_max

        # 追加: ステアリングレート制限関連のパラメータ
        self.max_steering_rate = max_steering_rate
        self.previous_steering = 0.0  # 前回のステア角

        # 追加: ay_maxによる速度制限の方式切り替え
        self.use_max_kappa_pred = use_max_kappa_pred
        # 既存の初期化
        self.current_prediction = None
        self.infeasibility_counter = 0
        self.last_solved_wp_id = 0
        self.current_control = np.zeros((self.nu*self.N))
        self.optimizer = osqp.OSQP()

        if not self.use_obstacle_avoidance:
            self.model.reference_path.update_simple_path_constraints(
                N,
                self.model.safety_margin)

    def update_v_max(self, v_max: float):
        self.input_constraints['umax'][0] = v_max

    def update_ay_max(self, ay_max: float):
        self.ay_max = ay_max

    def update_wp_id_offset(self, wp_id_offset: int):
        self.wp_id_offset = wp_id_offset

    def set_previous_steering(self, steer: float):
        self.previous_steering = steer

    def update_Q(self, Q: np.ndarray):
        self.Q = Q

    def update_R(self, R: np.ndarray):
        self.R = R

    def update_QN(self, QN: np.ndarray):
        self.QN = QN

    def _init_problem(self, N, safety_margin):
        """
        Initialize optimization problem for current time step with steering rate constraints.
        """
        # reset dynamic constraints
        if self.use_obstacle_avoidance and not self.use_path_constraints_topic:
            self.model.reference_path.reset_dynamic_constraints()
        # 既存の制約設定
        umin = self.input_constraints['umin']
        umax = self.input_constraints['umax']
        xmin = self.state_constraints['xmin']
        xmax = self.state_constraints['xmax']

        # Precompute common terms
        nx_N = self.nx * (N + 1)
        nu_N = self.nu * N

        # LTV System Matrices
        A = np.zeros((nx_N, nx_N))
        B = np.zeros((nx_N, nu_N))

        # Reference vector
        ur = np.zeros(nu_N)
        xr = np.zeros(nx_N)
        uq = np.zeros(N * self.nx)

        # Dynamic constraints
        xmin_dyn = np.kron(np.ones(N + 1), xmin)
        xmax_dyn = np.kron(np.ones(N + 1), xmax)
        umax_dyn = np.kron(np.ones(N), umax)

        # Get curvature predictions
        kappa_pred = np.tan(np.append(np.array(self.current_control[3::self.nu]), self.current_control[-1])) / self.model.length

        # Consider control delay without mutating self.model.wp_id (eliminating 40Hz discrete jump oscillation)
        start_wp_id = self.model.wp_id + self.wp_id_offset

        # Iterate over horizon
        for n in range(N):
            # Get waypoint information
            current_waypoint = self.model.reference_path.get_waypoint(start_wp_id + n)
            next_waypoint = self.model.reference_path.get_waypoint(start_wp_id + n + 1)
            delta_s = next_waypoint - current_waypoint
            kappa_ref = current_waypoint.kappa

            # Clip reference velocity
            v_ref = np.clip(current_waypoint.v_ref, self.input_constraints['umin'][0], self.input_constraints['umax'][0])

            # Compute LTV matrices
            f, A_lin, B_lin = self.model.linearize(v_ref, kappa_ref, delta_s)
            A[(n+1) * self.nx: (n+2)*self.nx, n * self.nx:(n+1)*self.nx] = A_lin
            B[(n+1) * self.nx: (n+2)*self.nx, n * self.nu:(n+1)*self.nu] = B_lin

            # Set reference
            ur[n*self.nu:(n+1)*self.nu] = [v_ref, kappa_ref]
            uq[n * self.nx:(n+1)*self.nx] = B_lin.dot([v_ref, kappa_ref]) - f

            # Constrain maximum speed based on road curvature
            vmax_dyn = np.sqrt(self.ay_max / (np.abs(kappa_ref) + 1e-6))
            umax_dyn[self.nu*n] = min(vmax_dyn, umax_dyn[self.nu*n])

        # Update path constraints
        if self.use_obstacle_avoidance and not self.use_path_constraints_topic and len(self.model.reference_path.map.obstacles) > 0:
            ub, lb, _ = self.model.reference_path.update_path_constraints(
                start_wp_id + 1,
                [self.model.temporal_state.x, self.model.temporal_state.y, self.model.temporal_state.psi],
                N, self.model.length, self.model.width, safety_margin)
        else:
            if self.model.reference_path.path_constraints is not None:
                ref_wp_id = (start_wp_id + 1) % len(self.model.reference_path.path_constraints[0])
                ub = self.model.reference_path.path_constraints[0][ref_wp_id]
                lb = self.model.reference_path.path_constraints[1][ref_wp_id]
                self.model.reference_path.border_cells.current_wp_id = ref_wp_id
            else:
                # path_constraints not set: use static waypoint ub_sm/lb_sm bounds
                # (already have safety margin applied via reset_dynamic_constraints)
                wps = self.model.reference_path.waypoints
                n_wps = len(wps)
                ub = np.array([self.model.reference_path.get_waypoint(start_wp_id + 1 + i).ub_sm for i in range(N)])
                lb = np.array([self.model.reference_path.get_waypoint(start_wp_id + 1 + i).lb_sm for i in range(N)])

            # Update safety margin if provided as argument and different from current value
            if self.model.safety_margin != safety_margin:
                safety_margin_diff = safety_margin - self.model.safety_margin
                ub -= safety_margin_diff
                lb += safety_margin_diff

                infeasible_index = ub < lb
                ub[infeasible_index] = 0.0
                lb[infeasible_index] = 0.0

        e_y0 = self.model.spatial_state.e_y
        e_psi0 = self.model.spatial_state.e_psi

        # dead_zone_filter: Suppress micro-noise chattering when vehicle is within ±3cm and ±0.5deg of target line
        if abs(e_y0) < 0.03:
            e_y0 = 0.0
        if abs(e_psi0) < 0.008:
            e_psi0 = 0.0

        # Latency-Compensated State Prediction (Smith-Predictor structure):
        # Predict spatial position (e_y_pred, e_psi_pred) after latency tau (80-120ms)
        # to ensure the MPC plans from where the vehicle WILL be when actuation takes effect.
        curr_wp_0 = self.model.reference_path.get_waypoint(self.model.wp_id)
        v_ref_0 = max(curr_wp_0.v_ref if (curr_wp_0 is not None and curr_wp_0.v_ref is not None) else 5.0, 1.0)
        tau_delay = 0.10  # 100ms system actuator + comm delay
        d_delay = v_ref_0 * tau_delay  # [m] distance traveled during delay
        kappa_prev_init = self.previous_steering
        kappa_ref_0 = curr_wp_0.kappa if curr_wp_0 is not None else 0.0

        e_y0_pred = e_y0 + d_delay * np.sin(e_psi0)
        e_psi0_pred = e_psi0 + d_delay * (kappa_prev_init - kappa_ref_0)

        # Guarantee kinematic feasibility over early horizon steps considering predicted position
        ds = self.model.reference_path.resolution
        for n in range(N):
            # Predict natural inertial lateral motion caused by current heading angle
            e_y_pred_step = e_y0_pred + (n + 1) * ds * np.sin(e_psi0_pred)
            margin = 0.4

            # Smoothly relax bounds if e_y0_pred or e_y_pred_step exceeds nominal corridor
            min_e_y = min(e_y0_pred, e_y_pred_step)
            max_e_y = max(e_y0_pred, e_y_pred_step)

            if min_e_y < lb[n]:
                decay = max(0.0, (15 - n) / 15.0)
                lb[n] = min(lb[n], (1.0 - decay) * lb[n] + decay * (min_e_y - margin))
            if max_e_y > ub[n]:
                decay = max(0.0, (15 - n) / 15.0)
                ub[n] = max(ub[n], (1.0 - decay) * ub[n] + decay * (max_e_y + margin))

        # Update dynamic state constraints
        xmin_dyn[self.nx::self.nx] = lb
        xmax_dyn[self.nx::self.nx] = ub
        # Initial state x0 constraint is strictly enforced via Aeq leq[0]/ueq[0] below (unconstrained in inequality block)
        xmin_dyn[0] = -np.inf
        xmax_dyn[0] = np.inf

        # [DEBUG] print diagnostics on each new infeasibility event
        if self.infeasibility_counter == 0:
            print(f'[MPC_DBG] wp_id={self.model.wp_id} e_y0={e_y0:.4f} e_psi0={e_psi0:.4f} '
                  f'ub[0]={float(ub[0] if hasattr(ub,"__len__") else ub):.4f} '
                  f'lb[0]={float(lb[0] if hasattr(lb,"__len__") else lb):.4f} '
                  f'N={N} lb_shape={getattr(lb,"shape","scalar")} ub_shape={getattr(ub,"shape","scalar")}')

        # Target lateral error e_y_ref = 0.0 (exact out-in-out reference path tracking)
        xr[self.nx::self.nx] = 0.0

        # Get equality matrix
        Ax = sparse.kron(sparse.eye(N + 1), -sparse.eye(self.nx)) + sparse.csc_matrix(A)
        Bu = sparse.csc_matrix(B)
        Aeq = sparse.hstack([Ax, Bu])

        # ステアリングレート（曲率変化率）制約の行列を構築（N個: u0 - kappa_prev, u1 - u0, ..., u_{N-1} - u_{N-2}）
        n_rate_constraints = N
        steering_rate_matrix = np.zeros((n_rate_constraints, nx_N + nu_N))

        # 第0行: u0 の位置に 1.0 (u0 - kappa_prev の制約用)
        steering_rate_matrix[0, nx_N + 1] = 1.0

        # 第1〜N-1行: u_i - u_{i-1} の差分制約
        for i in range(1, n_rate_constraints):
            steering_rate_matrix[i, nx_N + self.nu*(i-1) + 1] = -1.0
            steering_rate_matrix[i, nx_N + self.nu*i + 1] = 1.0

        # 制約行列の結合
        A_inequality = sparse.vstack([
            sparse.eye(nx_N + nu_N),  # 状態と入力の基本的な制約
            sparse.csc_matrix(steering_rate_matrix)  # ステアリングレート制約
        ])

        # 完全な制約行列
        A_full = sparse.vstack([Aeq, A_inequality], format='csc')

        # 境界制約の構築 (Latency-compensated predicted initial states e_y0_pred, e_psi0_pred applied)
        x0 = np.array(self.model.spatial_state[:])
        x0[0] = e_y0_pred
        x0[1] = e_psi0_pred
        leq = np.hstack([-x0, uq])
        ueq = leq.copy()
        # Strict initial state equality constraint
        leq[0] = -x0[0]
        ueq[0] = -x0[0]

        # 入力と状態の制約境界
        lineq_basic = np.hstack([xmin_dyn, np.kron(np.ones(N), umin)])
        uineq_basic = np.hstack([xmax_dyn, umax_dyn])

        # 空間ステップ ds (0.4m) と車速 v_ref に基づく物理的に正確な空間曲率変化率限界
        ds = self.model.reference_path.resolution
        max_kappa_change_list = []
        for n in range(N):
            curr_wp = self.model.reference_path.get_waypoint(start_wp_id + n)
            v_ref_n = max(curr_wp.v_ref if (curr_wp is not None and curr_wp.v_ref is not None) else 5.0, 1.0)
            dt_n = ds / v_ref_n
            max_kappa_change_n = (self.max_steering_rate / self.model.length) * dt_n
            max_kappa_change_list.append(max_kappa_change_n)

        # 直前の実ステアリング曲率 (previous_steering is already curvature kappa 1/m)
        kappa_prev = self.previous_steering

        lineq_rate = np.zeros(n_rate_constraints)
        uineq_rate = np.zeros(n_rate_constraints)

        # 第0行: kappa_prev - max_kappa_change[0] <= u0 <= kappa_prev + max_kappa_change[0]
        lineq_rate[0] = kappa_prev - max_kappa_change_list[0]
        uineq_rate[0] = kappa_prev + max_kappa_change_list[0]

        # 第1〜N-1行: -max_kappa_change[i] <= u_i - u_{i-1} <= max_kappa_change[i]
        for i in range(1, n_rate_constraints):
            lineq_rate[i] = -max_kappa_change_list[i]
            uineq_rate[i] = max_kappa_change_list[i]

        # 全ての境界を結合
        l = np.hstack([leq, lineq_basic, lineq_rate])
        u = np.hstack([ueq, uineq_basic, uineq_rate])

        # コスト行列
        P = sparse.block_diag([
            sparse.kron(sparse.eye(N), self.Q),
            self.QN,
            sparse.kron(sparse.eye(N), self.R)
        ], format='csc')

        q = np.hstack([
            -np.tile(np.diag(self.Q.toarray()), N) * xr[:-self.nx],
            -self.QN.dot(xr[-self.nx:]),
            -np.tile(np.diag(self.R.toarray()), N) * ur
        ])

        # オプティマイザの設定
        self.optimizer = osqp.OSQP()
        self.optimizer.setup(P=P, q=q, A=A_full, l=l, u=u, verbose=False, warm_start=True)

    def get_control(self) -> Tuple[np.ndarray, float]:
        """
        Get control signal given the current position of the car.
        """
        nx = self.nx
        nu = self.nu

        self.model.get_current_waypoint()

        N = min(self.N, self.model.reference_path.n_waypoints - self.model.wp_id) \
            if not self.model.reference_path.circular else self.N

        self.model.spatial_state = self.model.t2s(
            reference_state=self.model.temporal_state,
            reference_waypoint=self.model.current_waypoint)

        self._init_problem(N, self.model.safety_margin)

        try:
            dec = self.optimizer.solve()
            if dec.info.status != 'solved':
                print(f'[MPC] OSQP initial solve failed: status={dec.info.status}')
                # Phase 1: relax safety_margin (5 steps)
                for i in range(1, 6):
                    relaxed_safety_margin = self.model.safety_margin * ((5-i) / 5.0)
                    self._init_problem(N, relaxed_safety_margin)
                    dec = self.optimizer.solve()
                    if dec.info.status == 'solved':
                        if self.infeasibility_counter == 0:
                            if self.last_solved_wp_id != self.model.wp_id:
                                print(f"Relaxed safety margin by {relaxed_safety_margin} ({5-i}/5) to solve the problem")
                        break
                else:
                    # Phase 2: relax steer_rate_max (allow large corrections from large e_y / e_psi)
                    saved_steer_rate = self.max_steering_rate
                    for j in range(1, 5):
                        self.max_steering_rate = saved_steer_rate * (1.0 + j * 0.5)  # 1.5x, 2.0x, 2.5x, 3.0x
                        self._init_problem(N, 0.0)
                        dec = self.optimizer.solve()
                        if dec.info.status == 'solved':
                            print(f'[MPC] Solved with steer_rate_relaxation x{1.0 + j*0.5:.1f} at wp_id={self.model.wp_id}')
                            break
                    else:
                        self.max_steering_rate = saved_steer_rate
                        raise TypeError('OSQP solve failed after relaxation')
                    self.max_steering_rate = saved_steer_rate

            control_signals = np.array(dec.x[-N*nu:])
            use_control_signals = control_signals[1::2]

            # ステア角の計算と保存
            control_signals[1::2] = np.arctan(control_signals[1::2] * self.model.length)
            v = control_signals[0]
            delta = control_signals[1]

            # Note: self.previous_steering is synced from node via set_previous_steering(u[1])
            # using actual LPF-filtered output to ensure consistent steer rate limits

            # 予測の更新
            self.current_control = control_signals
            x = np.reshape(dec.x[:(N+1)*nx], (N+1, nx))
            self.current_prediction = self.update_prediction(x, N)

            u = np.array([v, delta])
            max_delta = np.max(np.abs(control_signals[1:len(control_signals)//3*2:2]))

            if self.infeasibility_counter > (N - 1):
                print(f'Problem solved after {self.infeasibility_counter} infeasible iterations')
            self.infeasibility_counter = 0
            self.last_solved_wp_id = self.model.wp_id

        except (TypeError, ValueError) as e:
            import traceback
            if self.infeasibility_counter == 0:
                print(f'[MPC] Exception in get_control: {type(e).__name__}: {e}')
                traceback.print_exc()
            id = nu * (self.infeasibility_counter + 1)
            if id + 2 < len(self.current_control):
                u = np.array(self.current_control[id:id+2])
                max_delta = np.abs(u[1])
            else:
                u = np.array([0.0, 0.0])
                max_delta = 0.0

            self.infeasibility_counter += 1

        if self.infeasibility_counter > (N - 1) and self.infeasibility_counter % 100 == 0:
            print('No control signal computed!')

        return u, max_delta

    def update_prediction(self, spatial_state_prediction, N):
        """
        Transform the predicted states to predicted x and y coordinates.
        Mainly for visualization purposes.
        :param spatial_state_prediction: list of predicted state variables
        :return: lists of predicted x and y coordinates
        """

        # Containers for x and y coordinates of predicted states
        x_pred, y_pred = [], []

        # Iterate over prediction horizon
        for n in range(2, N):
            # Get associated waypoint
            associated_waypoint = self.model.reference_path.\
                get_waypoint(self.model.wp_id+n)
            # Transform predicted spatial state to temporal state
            predicted_temporal_state = self.model.s2t(associated_waypoint,
                                            spatial_state_prediction[n, :])

            # Save predicted coordinates in world coordinate frame
            x_pred.append(predicted_temporal_state.x)
            y_pred.append(predicted_temporal_state.y)

        return x_pred, y_pred

    def show_prediction(self, ax):
        """
        Display predicted car trajectory on the provided axis.
        :param ax: Matplotlib axis object to plot on
        """

        if self.current_prediction is not None:
            # ax.scatter(self.current_prediction[0], self.current_prediction[1],
            #            c=PREDICTION, s=5)
            ax.plot(self.current_prediction[0], self.current_prediction[1], c=PREDICTION)
