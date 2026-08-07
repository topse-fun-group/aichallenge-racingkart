import numpy as np
from abc import abstractmethod
try:
    from abc import ABC
except:
    # for Python 2.7
    from abc import ABCMeta

    class ABC(object):
        __metaclass__ = ABCMeta
        pass
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches
import math

# Colors
CAR = '#F1C40F'
CAR_COLLIDING = '#FF0000'
CAR_OUTLINE = '#B7950B'


#########################
# Temporal State Vector #
#########################

class TemporalState:
    def __init__(self, x, y, psi):
        """
        Temporal State Vector containing car pose (x, y, psi)
        :param x: x position in global coordinate system | [m]
        :param y: y position in global coordinate system | [m]
        :param psi: yaw angle | [rad]
        """
        self.x = x
        self.y = y
        self.psi = psi

        self.members = ['x', 'y', 'psi']

    def __iadd__(self, other):
        """
        Overload Sum-Add operator.
        :param other: numpy array to be added to state vector
        """
        for state_id in range(len(self.members)):
            vars(self)[self.members[state_id]] += other[state_id]
        return self


########################
# Spatial State Vector #
########################

class SpatialState(ABC):
    """
    Spatial State Vector - Abstract Base Class.
    """

    @abstractmethod
    def __init__(self):
        self.members = None
        self.e_y = None
        self.e_psi = None

    def __getitem__(self, item):
        if isinstance(item, int):
            members = [self.members[item]]
        else:
            members = self.members[item]
        return [vars(self)[key] for key in members]

    def __setitem__(self, key, value):
        vars(self)[self.members[key]] = value

    def __len__(self):
        return len(self.members)

    def __iadd__(self, other):
        """
        Overload Sum-Add operator.
        :param other: numpy array to be added to state vector
        """

        for state_id in range(len(self.members)):
            vars(self)[self.members[state_id]] += other[state_id]
        return self

    def list_states(self):
        """
        Return list of names of all states.
        """
        return self.members


class SimpleSpatialState(SpatialState):
    def __init__(self, e_y=0.0, e_psi=0.0, t=0.0):
        """
        Simplified Spatial State Vector containing orthogonal deviation from
        reference path (e_y), difference in orientation (e_psi) and velocity
        :param e_y: orthogonal deviation from center-line | [m]
        :param e_psi: yaw angle relative to path | [rad]
        :param t: time | [s]
        """
        super(SimpleSpatialState, self).__init__()

        self.e_y = e_y
        self.e_psi = e_psi
        self.t = t

        self.members = ['e_y', 'e_psi', 't']


####################################
# Spatial Bicycle Model Base Class #
####################################

class SpatialBicycleModel(ABC):
    def __init__(self, reference_path, length, width, Ts):
        """
        Abstract Base Class for Spatial Reformulation of Bicycle Model.
        :param reference_path: reference path object to follow
        :param length: length of car in m
        :param width: width of car in m
        :param Ts: sampling time of model
        """

        # Precision
        self.eps = 1e-12

        # Car Parameters
        self.length = length
        self.width = width
        self.safety_margin = self._compute_safety_margin()

        # Reference Path
        self.reference_path = reference_path

        # Set initial distance traveled
        self.s = 0.0

        # Set sampling time
        self.Ts = Ts

        # Set initial waypoint ID
        self.wp_id = 0

        # Set initial waypoint
        self.current_waypoint = self.reference_path.waypoints[self.wp_id]

        # Declare spatial state variable | Initialization in sub-class
        self.spatial_state = None

        # Declare temporal state variable | Initialization in sub-class
        self.temporal_state = None

    def s2t(self, reference_waypoint, reference_state):
        """
        Convert spatial state to temporal state given a reference waypoint.
        :param reference_waypoint: waypoint object to use as reference
        :param reference_state: state vector as np.array to use as reference
        :return Temporal State equivalent to reference state
        """

        # Compute temporal state variables
        if isinstance(reference_state, np.ndarray):
            x = reference_waypoint.x - reference_state[0] * np.sin(
                reference_waypoint.psi)
            y = reference_waypoint.y + reference_state[0] * np.cos(
                reference_waypoint.psi)
            psi = reference_waypoint.psi + reference_state[1]
        elif isinstance(reference_state, SpatialState):
            x = reference_waypoint.x - reference_state.e_y * np.sin(
                reference_waypoint.psi)
            y = reference_waypoint.y + reference_state.e_y * np.cos(
                reference_waypoint.psi)
            psi = reference_waypoint.psi + reference_state.e_psi
        else:
            print('Reference State type not supported!')
            x, y, psi = None, None, None
            exit(1)

        return TemporalState(x, y, psi)

    def t2s(self, reference_waypoint, reference_state):
        """
        Convert temporal state to spatial state using Continuous Segment Projection
        to eliminate discrete 2.3-degree heading step jumps at waypoint transitions.
        :return Spatial State equivalent to reference state
        """
        if isinstance(reference_state, np.ndarray):
            cx, cy, cpsi = float(reference_state[0]), float(reference_state[1]), float(reference_state[2])
        elif isinstance(reference_state, TemporalState):
            cx, cy, cpsi = float(reference_state.x), float(reference_state.y), float(reference_state.psi)
        else:
            print('Reference State type not supported!')
            return SimpleSpatialState(0.0, 0.0, 0.0)

        # Use continuous segment projection around current self.wp_id (modulo n_base to avoid extension seam reversal)
        n_wps = len(self.reference_path.waypoints)
        if n_wps < 2:
            e_y = np.cos(reference_waypoint.psi) * (cy - reference_waypoint.y) - \
                  np.sin(reference_waypoint.psi) * (cx - reference_waypoint.x)
            e_psi = np.mod(cpsi - reference_waypoint.psi + math.pi, 2 * math.pi) - math.pi
            return SimpleSpatialState(e_y, e_psi, 0.0)

        n_base = getattr(self.reference_path, 'n_base_waypoints', n_wps) if getattr(self.reference_path, 'circular', False) else n_wps
        curr_id = getattr(self, 'wp_id', 0) % n_base
        best_dist_sq = float('inf')
        best_ey = 0.0
        best_epsi = 0.0

        # Test segments (curr_id-1, curr_id), (curr_id, curr_id+1), (curr_id+1, curr_id+2) modulo n_base
        p_car = np.array([cx, cy])
        for seg_offset in [-1, 0, 1]:
            idx1 = (curr_id + seg_offset) % n_base
            idx2 = (idx1 + 1) % n_base

            wp1 = self.reference_path.waypoints[idx1]
            wp2 = self.reference_path.waypoints[idx2]

            p1 = np.array([wp1.x, wp1.y])
            p2 = np.array([wp2.x, wp2.y])
            v_seg = p2 - p1
            v_len_sq = np.dot(v_seg, v_seg)
            if v_len_sq < 1e-12:
                continue

            t_factor = np.clip(np.dot(p_car - p1, v_seg) / v_len_sq, 0.0, 1.0)
            proj = p1 + t_factor * v_seg

            dist_sq = np.dot(p_car - proj, p_car - proj)
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                dpsi = np.mod(wp2.psi - wp1.psi + math.pi, 2 * math.pi) - math.pi
                proj_psi = wp1.psi + t_factor * dpsi

                norm_vec = np.array([-np.sin(proj_psi), np.cos(proj_psi)])
                best_ey = float(np.dot(p_car - proj, norm_vec))
                best_epsi = float(np.mod(cpsi - proj_psi + math.pi, 2 * math.pi) - math.pi)

        return SimpleSpatialState(best_ey, best_epsi, 0.0)

    def drive(self, u):
        """
        Drive.
        :param u: input vector containing [v, delta]
        """

        # Get input signals
        v, delta = u

        # Compute temporal state derivatives
        x_dot = v * np.cos(self.temporal_state.psi)
        y_dot = v * np.sin(self.temporal_state.psi)
        psi_dot = v / self.length * np.tan(delta)
        temporal_derivatives = np.array([x_dot, y_dot, psi_dot])

        # Update spatial state (Forward Euler Approximation)
        self.temporal_state += temporal_derivatives * self.Ts

        # Compute velocity along path
        s_dot = 1 / (1 - self.spatial_state.e_y * self.current_waypoint.kappa) \
                * v * np.cos(self.spatial_state.e_psi)

        # Update distance travelled along reference path
        self.s += s_dot * self.Ts

    def _compute_safety_margin(self):
        """
        Compute safety margin for car if modeled by its center of gravity.
        """

        # Model ellipsoid around the car
        # safety_margin = self.width / 2.0  # Physical half-width: 1.45/2 = 0.725m (was width/sqrt(2) = 1.025m)
        safety_margin = self.width / 4.0  # 0.25m margin unblocking inner corner bounds (lb_sm)
        return safety_margin

    def get_current_waypoint(self):
        """
        Get closest waypoint on reference path based on car's current location.
        """

        # Compute cumulative path length
        length_cum = np.cumsum(self.reference_path.segment_lengths)
        # Get first index with distance larger than distance traveled by car
        # so far
        greater_than_threshold = length_cum > self.s
        next_wp_id = greater_than_threshold.searchsorted(True)

        # check end of path
        if next_wp_id == len(length_cum):
            self.wp_id = len(length_cum) - 1
            self.current_waypoint = self.reference_path.waypoints[self.wp_id]
            return

        # Get previous index
        prev_wp_id = next_wp_id - 1

        # Get distance traveled for both enclosing waypoints
        s_next = length_cum[next_wp_id]
        s_prev = length_cum[prev_wp_id]

        if np.abs(self.s - s_next) < np.abs(self.s - s_prev):
            self.wp_id = next_wp_id
            self.current_waypoint = self.reference_path.waypoints[next_wp_id]
        else:
            self.wp_id = prev_wp_id
            self.current_waypoint = self.reference_path.waypoints[prev_wp_id]

    def get_closest_waypoint_global(self, x, y):
        """
        Get the index of the closest waypoint to given x, y coordinates across
        a wide forward horizon (-10 to +60 waypoints = ~24m) for V2X obstacle matching.
        """
        n_wps = len(self.reference_path.waypoints)
        if n_wps == 0:
            return 0
        n_base = getattr(self.reference_path, 'n_base_waypoints', n_wps) if getattr(self.reference_path, 'circular', False) else n_wps
        curr_id = getattr(self, 'wp_id', 0) % n_base

        # Forward extended search window (-10 to +60 waypoints = 24m)
        window_indices = [(curr_id + i) % n_base for i in range(-10, 61)]
        best_id = curr_id
        min_dist_sq = float('inf')

        for idx in window_indices:
            wp = self.reference_path.waypoints[idx]
            dist_sq = (wp.x - x)**2 + (wp.y - y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_id = idx

        return best_id

    def get_closest_waypoint(self, x, y):
        """
        Get the index of the closest waypoint to the given x, y coordinates
        using a local window around current self.wp_id to prevent false jumps
        to adjacent track segments.
        :param x: x coordinate
        :param y: y coordinate
        :return: Index of the closest waypoint
        """
        n_wps = len(self.reference_path.waypoints)
        if n_wps == 0:
            return 0

        n_base = getattr(self.reference_path, 'n_base_waypoints', n_wps) if getattr(self.reference_path, 'circular', False) else n_wps

        # If wp_id is not yet initialized or zero, fallback to full search once
        curr_id = getattr(self, 'wp_id', 0) % n_base
        if not hasattr(self, '_wp_id_initialized') or not self._wp_id_initialized:
            distances_sq = (np.array([wp.x for wp in self.reference_path.waypoints[:n_base]]) - x)**2 + \
                           (np.array([wp.y for wp in self.reference_path.waypoints[:n_base]]) - y)**2
            self._wp_id_initialized = True
            return int(np.argmin(distances_sq))

        # Local window search (-10 to +25 waypoints around curr_id, wrapped modulo n_base)
        window_indices = [(curr_id + i) % n_base for i in range(-10, 26)]
        best_id = curr_id
        min_dist_sq = float('inf')

        for idx in window_indices:
            wp = self.reference_path.waypoints[idx]
            dist_sq = (wp.x - x)**2 + (wp.y - y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_id = idx

        return best_id

    def get_s_at_waypoint(self, wp_id):
        """
        Calculate the distance s along the reference path corresponding to the
        waypoint.
        :param wp_id: waypoint id
        :return: Distance s along the reference path
        """
        # Compute cumulative path length
        length_cum = np.cumsum(self.reference_path.segment_lengths)

        # Distance s at the closest waypoint
        s_at_closest_wp = length_cum[wp_id]

        return s_at_closest_wp

    def show(self, is_colliding: bool, ax):
        """
        Display car on the provided axis.
        :param ax: Matplotlib axis object to plot on
        """

        # Get car's center of gravity
        cog = (self.temporal_state.x, self.temporal_state.y)
        # Get current angle with respect to x-axis
        yaw = np.rad2deg(self.temporal_state.psi)

        facecolor = CAR_COLLIDING if is_colliding else CAR

        # Draw rectangle
        car = plt_patches.Rectangle(cog, width=self.length, height=self.width,
                                    angle=yaw, facecolor=facecolor,
                                    edgecolor=CAR_OUTLINE, zorder=20)

        # Shift center rectangle to match center of the car
        car.set_x(car.get_x() - (self.length / 2 *
                                 np.cos(self.temporal_state.psi) -
                                 self.width / 2 *
                                 np.sin(self.temporal_state.psi)))
        car.set_y(car.get_y() - (self.width / 2 *
                                 np.cos(self.temporal_state.psi) +
                                 self.length / 2 *
                                 np.sin(self.temporal_state.psi)))

        # Add rectangle to provided axis
        ax.add_patch(car)

    @abstractmethod
    def get_spatial_derivatives(self, state, input, kappa):
        pass

    @abstractmethod
    def linearize(self, v_ref, kappa_ref, delta_s):
        pass


#################
# Bicycle Model #
#################

class BicycleModel(SpatialBicycleModel):
    def __init__(self, reference_path, length, width, Ts):
        """
        Simplified Spatial Bicycle Model. Spatial Reformulation of Kinematic
        Bicycle Model. Uses Simplified Spatial State.
        :param reference_path: reference path model is supposed to follow
        :param length: length of the car in m
        :param width: with of the car in m
        :param Ts: sampling time of model in s
        """

        # Initialize base class
        super(BicycleModel, self).__init__(reference_path, length=length,
                                           width=width, Ts=Ts)

        # Initialize spatial state
        self.spatial_state = SimpleSpatialState()

        # Number of spatial state variables
        self.n_states = len(self.spatial_state)

        # Initialize temporal state
        self.temporal_state = self.s2t(reference_state=self.spatial_state,
                                       reference_waypoint=self.current_waypoint)

    def update_reference_path(self, reference_path):
        # Update Reference Path
        self.reference_path = reference_path

        # Update the current waypoint based on the new reference path
        self.wp_id = self.get_closest_waypoint(self.temporal_state.x, self.temporal_state.y)

        # Update the distance s along the reference path based on the closest waypoint
        self.s = self.get_s_at_waypoint(self.wp_id)

    def update_states(self, x, y, psi):
        self.temporal_state.x = x
        self.temporal_state.y = y
        self.temporal_state.psi = psi

        # Update the current waypoint based on the new reference path
        self.wp_id = self.get_closest_waypoint(self.temporal_state.x, self.temporal_state.y)

        # Update the distance s along the reference path based on the closest waypoint
        self.s = self.get_s_at_waypoint(self.wp_id)

    def get_temporal_derivatives(self, state, input, kappa):
        """
        Compute relevant temporal derivatives needed for state update.
        :param state: state vector for which to compute derivatives
        :param input: input vector
        :param kappa: curvature of corresponding waypoint
        :return: temporal derivatives of distance, angle and velocity
        """

        # Get state and input variables
        e_y, e_psi, t = state
        v, delta = input

        # Compute velocity along path
        s_dot = 1 / (1 - (e_y * kappa)) * v * np.cos(e_psi)

        # Compute yaw angle rate of change
        psi_dot = v / self.length * np.tan(delta)

        return s_dot, psi_dot

    def get_spatial_derivatives(self, state, input, kappa):
        """
        Compute spatial derivatives of all state variables for update.
        :param state: state vector for which to compute derivatives
        :param input: input vector
        :param kappa: curvature of corresponding waypoint
        :return: numpy array with spatial derivatives for all state variables
        """

        # Get state and input variables
        e_y, e_psi, t = state
        v, delta = input

        # Compute temporal derivatives
        s_dot, psi_dot = self.get_temporal_derivatives(state, input, kappa)

        # Compute spatial derivatives
        d_e_y_d_s = v * np.sin(e_psi) / s_dot
        d_e_psi_d_s = psi_dot / s_dot - kappa
        d_t_d_s = 1 / s_dot

        return np.array([d_e_y_d_s, d_e_psi_d_s, d_t_d_s])

    def linearize(self, v_ref, kappa_ref, delta_s):
        """
        Linearize the system equations around provided reference values.
        :param v_ref: velocity reference around which to linearize
        :param kappa_ref: kappa of waypoint around which to linearize
        :param delta_s: distance between current waypoint and next waypoint
         """

        ###################
        # System Matrices #
        ###################

        # Construct Jacobian Matrix
        a_1 = np.array([1, delta_s, 0])
        a_2 = np.array([-kappa_ref ** 2 * delta_s, 1, 0])

        b_1 = np.array([0, 0])
        b_2 = np.array([0, delta_s])

        # Handle v_ref == 0 case
        if v_ref == 0:
            a_3 = np.array([0, 0, 1])
            b_3 = np.array([0, 0])
            f = np.array([0.0, 0.0, 0.0])
        else:
            a_3 = np.array([-kappa_ref / v_ref * delta_s, 0, 1])
            b_3 = np.array([-1 / (v_ref ** 2) * delta_s, 0])
            f = np.array([0.0, 0.0, 1 / v_ref * delta_s])

        A = np.stack((a_1, a_2, a_3), axis=0)
        B = np.stack((b_1, b_2, b_3), axis=0)

        return f, A, B
