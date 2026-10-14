from __future__ import annotations

from dexhand_servo.control.snapshot import ServoReading
from dexhand_servo.control.trace import TraceRecorder, write_trace_csv


def reading(position: int) -> ServoReading:
    return ServoReading(1, position, 100, position, 0, 5.0, 6.0, 30, 0, 0, 0)


def test_stair_and_stall_metrics(tmp_path) -> None:
    rec = TraceRecorder("t", 0.0, {1: 510.0}, {1: "j"})
    # goal advances 2 steps per cycle; the joint sticks for two cycles, then catches up
    for k, (setpoint, position) in enumerate([(500, 500), (502, 500), (504, 500), (506, 504), (508, 507),
                                               (510, 509), (510, 510)]):
        rec.add(k * 0.02, 1, setpoint, setpoint, reading(position), speed_register=60)
    arrival = rec.arrivals()[0]
    assert arrival.max_goal_step == 2
    assert arrival.stalled_pct == 40.0  # 2 of the 5 advancing cycles had no movement
    assert "max goal step 2" in rec.summary_line()
    header = write_trace_csv(rec, tmp_path / "t.csv").read_text().splitlines()[0]
    assert header.endswith("speed_register")
