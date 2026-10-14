from __future__ import annotations

import pytest

from dexhand_servo.config import JOINTS, JointConfig, validate_joints


def test_shipped_joint_table_is_valid() -> None:
    assert validate_joints(JOINTS) == []


@pytest.mark.parametrize(
    "joints,fragment",
    [
        ({0: JointConfig("a")}, "1..253"),
        ({1: JointConfig(" ")}, "name is empty"),
        ({1: JointConfig("a"), 2: JointConfig("a")}, "already used"),
        ({1: JointConfig("a", min_position=100)}, "both"),
        ({1: JointConfig("a", min_position=500, max_position=400)}, "min_position < max_position"),
        ({1: JointConfig("a", min_position=-1, max_position=400)}, "min_position < max_position"),
        ({1: JointConfig("a", min_position=100, max_position=1024)}, "min_position < max_position"),
        ({1: JointConfig("a", min_position=100.5, max_position=400)}, "integers"),
    ],
)
def test_invalid_entries_are_reported(joints, fragment: str) -> None:
    problems = validate_joints(joints)
    assert len(problems) == 1 and fragment in problems[0], problems


def test_tbd_names_may_repeat_and_limits_are_optional() -> None:
    joints = {1: JointConfig("TBD"), 2: JointConfig("TBD"), 3: JointConfig("thumb_cmc", 120, 640)}
    assert validate_joints(joints) == []
    assert joints[3].has_travel_limits and not joints[1].has_travel_limits


def test_cli_refuses_inconsistent_joint_table(monkeypatch) -> None:
    from dexhand_servo import cli

    monkeypatch.setattr(cli, "JOINTS", {1: JointConfig("a", min_position=900, max_position=100)})
    assert cli.main(["list-ports"]) == 2


def test_stream_speed_register() -> None:
    from dataclasses import replace

    from dexhand_servo.config import MotionConfig

    m = MotionConfig(stream_speed_mode="dynamic", stream_speed_gain=1.3, stream_speed_floor=50,
                     stream_speed_register=1000)
    assert [m.stream_speed(v) for v in (0, 30, 100, -200, 5000)] == [50, 50, 130, 260, 1000]
    assert replace(m, stream_speed_mode="fixed").stream_speed(10) == 1000
