"""Arming (torque on, holding the present position) and disarming of single joints.

Goal is written before torque so that, whether or not a goal write implicitly
enables torque on this firmware, the joint only ever holds where it already is.
The policy refuses goals to disarmed joints, so no other code path can switch
torque on.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..config import JointConfig, MotionConfig
from ..registers import REG_TORQUE_ENABLE
from ..safe_bus import SafeBus, UnsafeOperationError
from .policy import REG_GOAL_BLOCK, MotionPolicy, goal_block
from .snapshot import STATUS_BLOCK_ADDR, STATUS_BLOCK_LEN, ServoReading, decode_status_block, describe_error


@dataclass(frozen=True)
class ArmResult:
    ok: bool
    reason: str
    reading: ServoReading | None = None


def read_status(bus: SafeBus, servo_id: int) -> ServoReading | None:
    t = bus.read(servo_id, STATUS_BLOCK_ADDR, STATUS_BLOCK_LEN)
    if not t.ok or len(t.data) != STATUS_BLOCK_LEN:
        return None
    return decode_status_block(t.data, t.error)


def arm_joint(
    bus: SafeBus, policy: MotionPolicy, servo_id: int, joint: JointConfig, motion: MotionConfig
) -> ArmResult:
    if not joint.has_travel_limits:
        return ArmResult(False, "no travel limits configured")
    assert joint.min_position is not None and joint.max_position is not None
    reading = read_status(bus, servo_id)
    if reading is None:
        return ArmResult(False, "no clean status reply")
    if reading.error_byte or reading.status:
        detail = describe_error(reading.error_byte) or f"status 0x{reading.status:02X}"
        return ArmResult(False, f"servo reports an error ({detail})", reading)
    if not joint.min_position <= reading.position <= joint.max_position:
        return ArmResult(
            False,
            f"position {reading.position} outside travel [{joint.min_position}, {joint.max_position}]; "
            "move the joint back by hand with torque off",
            reading,
        )

    policy.add_armed(servo_id)
    hold = reading.position
    policy.seed_goal(servo_id, hold)
    speed = min(motion.arm_speed, policy.limits[servo_id].speed_cap)
    try:
        goal = bus.write_block(servo_id, REG_GOAL_BLOCK, goal_block(hold, speed))
        torque = bus.write_byte(servo_id, REG_TORQUE_ENABLE, 1) if goal.ok else None
    except UnsafeOperationError as exc:
        disarm_joint(bus, policy, servo_id)
        return ArmResult(False, f"refused by motion policy: {exc}", reading)
    check = read_status(bus, servo_id) if torque is not None and torque.ok else None

    problem = None
    if not goal.ok:
        problem = f"goal write failed ({goal.result.value})"
    elif torque is None or not torque.ok:
        problem = "torque write failed"
    elif check is None:
        problem = "no read-back after arming"
    elif check.torque != 1:
        problem = "torque did not switch on"
    elif check.goal != hold:
        problem = f"goal register reads {check.goal}, expected {hold}"
    elif abs(check.position - hold) > motion.arm_hold_tolerance:
        problem = f"joint moved to {check.position} while arming (held {hold})"
    if problem:
        disarm_joint(bus, policy, servo_id)
        return ArmResult(False, problem, check or reading)
    return ArmResult(True, f"armed, holding {hold}", check)


def disarm_joint(bus: SafeBus, policy: MotionPolicy, servo_id: int) -> bool:
    """Remove from the armed set first (no goal can follow), then torque off and verify."""
    policy.remove_armed(servo_id)
    bus.write_byte(servo_id, REG_TORQUE_ENABLE, 0)
    check = read_status(bus, servo_id)
    return check is not None and check.torque == 0
