from __future__ import annotations

from dataclasses import replace

from dexhand_servo.config import MotionConfig
from dexhand_servo.control.snapshot import ServoReading
from dexhand_servo.control.supervisor import JointInput, Supervisor

BASE = ServoReading(torque=1, goal=500, goal_speed=225, position=500, speed=0, load_pct=5.0, voltage=6.0,
                    temperature=30, status=0, moving=0, error_byte=0)
MOTION = MotionConfig(heartbeat_freeze_s=0.5, heartbeat_trip_s=2.0)
# Inputs derive from the configured thresholds, so tuning config.py does not break these tests.
SOFT_LOAD = (MOTION.load_trip_pct + MOTION.load_hard_pct) / 2  # above trip, below hard
HARD_LOAD = MOTION.load_hard_pct + 5


def joint(reading=BASE, armed=True, setpoint=500.0, failures=0, last_ok=0.0) -> JointInput:
    return JointInput(reading, failures, last_ok, armed, setpoint if armed else None)


def run(sup, readings_over_time, armed=True, setpoint=500.0, dt=0.02):
    """Feed one joint's readings at fixed dt; return (time of first trip, trip)."""
    t = 0.0
    for reading in readings_over_time:
        result = sup.evaluate(t, {1: joint(reading, armed, setpoint, last_ok=t)})
        if result.trips:
            return t, result.trips[0]
        t += dt
    return None, None


def test_load_trip_needs_persistence() -> None:
    sup = Supervisor(MOTION)
    short = int(MOTION.load_trip_s / 0.02) - 3
    t, trip = run(sup, [replace(BASE, load_pct=SOFT_LOAD)] * short)
    assert trip is None
    sup.reset()
    t, trip = run(sup, [replace(BASE, load_pct=SOFT_LOAD)] * 60)
    assert trip.scope == "joint" and "load" in trip.reason
    assert MOTION.load_trip_s - 0.01 <= t <= MOTION.load_trip_s + 0.03


def test_hard_load_trips_fast_and_sign_is_ignored() -> None:
    t, trip = run(Supervisor(MOTION), [replace(BASE, load_pct=-HARD_LOAD)] * 30)
    assert trip is not None and t <= MOTION.load_hard_s + 0.02


def test_disarmed_load_only_warns() -> None:
    result = Supervisor(MOTION).evaluate(0.0, {1: joint(replace(BASE, torque=0, load_pct=90.0), armed=False)})
    assert not result.trips and result.warnings[1]


def test_tracking_error() -> None:
    lagging = replace(BASE, position=500 - MOTION.tracking_error_steps - 5)
    t, trip = run(Supervisor(MOTION), [lagging] * 60, setpoint=500.0)
    assert trip is not None and "tracking" in trip.reason
    assert MOTION.tracking_error_s - 0.01 <= t <= MOTION.tracking_error_s + 0.03


def test_goal_mismatch_detects_lost_sync_writes() -> None:
    t, trip = run(Supervisor(MOTION), [replace(BASE, goal=480, position=500)] * 40, setpoint=500.0)
    assert trip is not None and "goal register" in trip.reason


def test_stall_under_load() -> None:
    sup = Supervisor(MOTION)
    t, trip = None, None
    for i in range(40):
        now = i * 0.02
        stall_load = min(MOTION.stall_load_pct + 5, MOTION.load_trip_pct - 1)
        result = sup.evaluate(now, {1: joint(replace(BASE, load_pct=stall_load, goal=500 + i), setpoint=500.0 + i,
                                             last_ok=now)})
        if result.trips:
            trip = result.trips[0]
            break
    assert trip is not None and "stalled" in trip.reason


def test_torque_state_mismatch_trips_all() -> None:
    sup = Supervisor(MOTION)
    dropped = sup.evaluate(0.0, {1: joint(replace(BASE, torque=0))})
    assert dropped.trips[0].scope == "all"
    unexpected = sup.evaluate(0.0, {1: joint(replace(BASE, torque=1), armed=False)})
    assert unexpected.trips[0].scope == "all"


def test_voltage_error_bit_trips_all() -> None:
    result = Supervisor(MOTION).evaluate(0.0, {1: joint(replace(BASE, error_byte=1))})
    assert result.trips[0].scope == "all"
    result = Supervisor(MOTION).evaluate(0.0, {1: joint(replace(BASE, error_byte=32))})
    assert result.trips[0].scope == "joint"


def test_supply_window() -> None:
    sup = Supervisor(MOTION)
    sag = replace(BASE, voltage=5.3)
    t, trip = run(sup, [sag] * 20)
    assert trip is not None and trip.scope == "all" and 0.19 <= t <= 0.22
    idle = Supervisor(MOTION).evaluate(0.0, {1: joint(replace(sag, torque=0), armed=False)})
    assert not idle.trips and idle.blocking, "disarmed: no trip, but arming is blocked"


def test_comm_loss() -> None:
    sup = Supervisor(MOTION)
    result = sup.evaluate(1.0, {1: joint(None, failures=3, last_ok=0.9)})
    assert result.comm_lost and result.trips[0].scope == "all"
    idle = sup.evaluate(1.0, {1: joint(None, armed=False, failures=3, last_ok=0.9)})
    assert idle.comm_lost and not idle.trips


def test_heartbeat_freeze_then_trip() -> None:
    sup = Supervisor(MOTION)
    assert sup.evaluate(0.0, {1: joint()}, heartbeat_age=0.6).freeze_motion
    assert sup.evaluate(0.0, {1: joint()}, heartbeat_age=2.5).trips[0].scope == "all"


def test_temperature() -> None:
    t, trip = run(Supervisor(MOTION), [replace(BASE, temperature=66)] * 60)
    assert trip is not None and 0.99 <= t <= 1.03
    warm = Supervisor(MOTION).evaluate(0.0, {1: joint(replace(BASE, temperature=56))})
    assert not warm.trips and warm.warnings[1]


def test_sustained_load_warning_when_parked_short_of_target() -> None:
    sup = Supervisor(MOTION)
    parked = replace(BASE, position=520, load_pct=MOTION.hold_load_warn_pct + 5)
    warned_at = None
    for i in range(int((MOTION.hold_warn_s + 1) / 0.05)):
        result = sup.evaluate(i * 0.05, {1: joint(parked, setpoint=500.0, last_ok=i * 0.05)})
        if result.sustained_load and warned_at is None:
            warned_at = i * 0.05
    assert warned_at is not None and MOTION.hold_warn_s <= warned_at <= MOTION.hold_warn_s + 0.1
    assert not result.trips, "a parked joint below the trip thresholds is a warning, not a protective stop"


def test_no_sustained_load_warning_while_moving_or_on_target() -> None:
    sup = Supervisor(MOTION)
    for i in range(200):  # setpoint moving
        result = sup.evaluate(i * 0.05, {1: joint(replace(BASE, position=480, load_pct=40.0, goal=500 + i),
                                                  setpoint=500.0 + i, last_ok=i * 0.05)})
        assert not result.sustained_load
    sup = Supervisor(MOTION)
    for i in range(200):  # on target, holding against a load: fine
        result = sup.evaluate(i * 0.05, {1: joint(replace(BASE, position=500, load_pct=40.0), last_ok=i * 0.05)})
        assert not result.sustained_load


def test_free_running_single_write_pauses_tracking_and_stall_only() -> None:
    sup = Supervisor(MOTION)
    far = replace(BASE, goal=650, position=500, load_pct=MOTION.stall_load_pct + 5)
    for i in range(100):
        now = i * 0.02
        result = sup.evaluate(now, {1: JointInput(far, 0, now, True, 650.0, free_running=True)})
        assert not result.trips, result.trips
    t, trip = None, None
    for i in range(100, 200):  # free running ends, joint still far from its goal
        now = i * 0.02
        result = sup.evaluate(now, {1: JointInput(replace(far, load_pct=5.0), 0, now, True, 650.0)})
        if result.trips:
            t, trip = now, result.trips[0]
            break
    assert trip is not None and "tracking" in trip.reason
    assert t - 2.0 >= MOTION.tracking_error_s - 0.01, "the debounce starts when free running ends"

    hot = Supervisor(MOTION)
    loaded = replace(far, load_pct=MOTION.load_hard_pct + 5)
    tripped = [hot.evaluate(i * 0.02, {1: JointInput(loaded, 0, i * 0.02, True, 650.0, free_running=True)}).trips
               for i in range(20)]
    assert any(tripped), "load protection stays active"
    lost = Supervisor(MOTION)
    mismatch = replace(BASE, goal=480)
    tripped = [lost.evaluate(i * 0.02, {1: JointInput(mismatch, 0, i * 0.02, True, 650.0, free_running=True)}).trips
               for i in range(40)]
    assert any(t and "goal register" in t[0].reason for t in tripped), "a lost single write is still detected"
