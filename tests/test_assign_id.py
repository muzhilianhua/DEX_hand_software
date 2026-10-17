from __future__ import annotations

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.assign_id import AssignmentAborted, assign_id


def always(answer: bool):
    return lambda snapshot: answer


@pytest.mark.parametrize("reply_from_new_id", [False, True])
def test_single_servo_gets_new_id(open_bus, cfg, reply_from_new_id: bool) -> None:
    servo = FakeServo(1, reply_from_new_id=reply_from_new_id)
    fake = FakeBus([servo, FakeServo(2)])  # an already-assigned servo may stay connected
    result = assign_id(open_bus(fake), 1, 3, cfg, always(True))

    assert result.success, result.steps
    assert servo.current_id == 3
    assert servo.regs[48] == 1
    assert fake.writes() == [(1, 48, 0), (1, 5, 3), (3, 48, 1)]


def test_refuses_when_several_servos_share_from_id(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1, position=300), FakeServo(1, position=700)])
    with pytest.raises(AssignmentAborted, match="exactly one servo"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []


def test_refuses_when_target_id_taken(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1), FakeServo(3)])
    with pytest.raises(AssignmentAborted, match="not free"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []


def test_refuses_without_operator_confirmation(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1)])
    seen = {}

    def confirm(snapshot):
        seen.update(snapshot)
        return False

    with pytest.raises(AssignmentAborted, match="did not confirm"):
        assign_id(open_bus(fake), 1, 3, cfg, confirm)
    assert seen["present_voltage"] == 6.0 and seen["to_id"] == 3
    assert fake.writes() == []


@pytest.mark.parametrize("from_id,to_id", [(1, 1), (1, 254), (1, 0), (300, 2)])
def test_rejects_invalid_ids(open_bus, cfg, from_id: int, to_id: int) -> None:
    fake = FakeBus([FakeServo(1)])
    with pytest.raises(AssignmentAborted):
        assign_id(open_bus(fake), from_id, to_id, cfg, always(True))
    assert fake.received == []


def test_refuses_servo_that_is_not_writable_model(open_bus, cfg) -> None:
    servo = FakeServo(1)
    servo.regs[2], servo.regs[3] = 0, 10  # identifies as HLS3606
    fake = FakeBus([servo])
    with pytest.raises(AssignmentAborted, match="HLS3606"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []
