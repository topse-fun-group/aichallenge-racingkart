import pytest
import numpy as np
from multi_purpose_mpc_ros.modes.v2x_mode_manager import V2XModeManager, V2XStateOutput
from multi_purpose_mpc_ros.modes.stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryOutput


def test_v2x_mode_manager_initialization():
    manager = V2XModeManager(vehicle_id="d2")
    assert manager.mode == "NORMAL"
    assert manager.vehicle_radius == 1.0
    assert manager.speed_limit == float('inf')


def test_v2x_mode_manager_overtaking_lock():
    manager = V2XModeManager(vehicle_id="d2")
    manager.lock_overtaking(lock_until_sec=100.0, radius=0.65)
    assert manager.mode == "OVERTAKING"
    assert manager.vehicle_radius == 0.65
    assert manager.speed_limit == float('inf')


def test_v2x_mode_manager_null_safety():
    manager = V2XModeManager(vehicle_id="d2")
    res = manager.update(
        now_sec=10.0,
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_speed_mps=0.0,
        ego_wp_id=0,
        tracker=None,
        reference_path=None,
        car=None,
        v2x_cfg=None,
        mpc_v_max=15.0,
        vehicle_radius_normal=1.0
    )
    assert res.mode == "NORMAL"


def test_v2x_mode_manager_stationary_approach_speed_limit():
    class DummyV2XCfg:
        follow_distance_start = 15.0
        follow_distance_brake = 5.0
        v_min_safe = 8.0
        ttc_threshold = 1.5
        forward_cos_threshold = 0.5
        overtake_patience = 3.0
        overtake_gap_min = 10.0
        overtake_clearance = 8.0
        vehicle_radius_overtake = 0.65

    manager = V2XModeManager(vehicle_id="d1")
    manager.mode = "OVERTAKING"

    # When no lead vehicle is ahead, OVERTAKING mode completes (back to NORMAL)
    res = manager.update(
        now_sec=10.0,
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_speed_mps=3.0,
        ego_wp_id=10,
        tracker=None,
        reference_path=None,
        car=None,
        v2x_cfg=DummyV2XCfg(),
        mpc_v_max=15.0,
        vehicle_radius_normal=1.0
    )
    assert res.mode == "NORMAL"
    assert res.speed_limit == float('inf')


def test_stuck_recovery_manager_initialization():
    manager = StuckRecoveryManager(vehicle_id="d2")
    assert manager.state == "NORMAL"
    assert manager.retry_count == 0


def test_stuck_recovery_manager_collision_trigger():
    class DummyStuckCfg:
        enable_stuck_recovery = True
        stuck_velocity_threshold = 0.25
        stuck_time_threshold = 1.0
        reverse_duration = 2.2
        stop_duration = 0.4
        reverse_speed = 2.5
        reverse_steer_angle = 0.35

    manager = StuckRecoveryManager(vehicle_id="d2")
    stuck_cfg = DummyStuckCfg()

    # Step 1: collision candidate
    out1 = manager.update(
        now_sec=1.0,
        v_curr=0.0,
        u_cmd=(1.0, 0.0),
        is_colliding=True,
        enable_control=True,
        has_launched=True,
        odom=None,
        tracker=None,
        car=None,
        mpc=None,
        stuck_cfg=stuck_cfg
    )
    assert out1.state == "NORMAL"

    # Step 2: 0.5s later during collision -> BRAKE_BEFORE_REVERSE
    out2 = manager.update(
        now_sec=1.6,
        v_curr=0.0,
        u_cmd=(1.0, 0.0),
        is_colliding=True,
        enable_control=True,
        has_launched=True,
        odom=None,
        tracker=None,
        car=None,
        mpc=None,
        stuck_cfg=stuck_cfg
    )
    assert out2.state == "BRAKE_BEFORE_REVERSE"
    assert out2.override_control is True
