from __future__ import annotations

import pytest

from dexhand_servo.control.trajectory import (
    Keyframe,
    RateLimiter,
    compress_keyframes,
    min_jerk,
    plan_move,
    plan_playback,
)


def test_rate_limiter_respects_speed_and_acceleration() -> None:
    limiter = RateLimiter(400, max_speed=150, acceleration=600, lo=300, hi=700)
    limiter.set_target(650)
    dt, previous, previous_v = 0.02, 400.0, 0.0
    for _ in range(400):
        position = limiter.step(dt)
        velocity = (position - previous) / dt
        assert abs(velocity) <= 150 + 1e-6
        assert abs(velocity - previous_v) <= 600 * dt + 1e-6 or position == 650
        previous, previous_v = position, velocity
    assert position == 650


def test_rate_limiter_clamps_to_travel() -> None:
    limiter = RateLimiter(400, 150, 600, 300, 700)
    limiter.set_target(900)
    assert limiter.target == 700
    for _ in range(500):
        limiter.step(0.02)
    assert limiter.position == 700


def test_min_jerk_endpoints() -> None:
    assert min_jerk(0) == 0 and min_jerk(1) == 1 and min_jerk(0.5) == pytest.approx(0.5)


def test_sync_move_arrives_together_under_speed_limits() -> None:
    plan = plan_move({1: 400, 2: 600}, {1: 600, 2: 590}, {1: 150, 2: 150}, now=0.0, min_duration=0.3, label="t")
    assert plan.end == pytest.approx(1.875 * 200 / 150)
    targets, progress, done = plan.sample(plan.end)
    assert done and progress == 1.0 and targets == {1: 600, 2: 590}
    mid, _, _ = plan.sample(plan.end / 2)
    assert mid[1] == pytest.approx(500) and mid[2] == pytest.approx(595)


def test_compress_keeps_endpoints_and_significant_moves() -> None:
    frames = [Keyframe(i * 0.02, {1: 400 + (i // 10) * 5}) for i in range(50)]
    kept = compress_keyframes(frames, tolerance=3)
    assert kept[0] is frames[0] and kept[-1] is frames[-1]
    assert len(kept) < len(frames) / 4


def test_playback_starts_with_approach_from_actual_position() -> None:
    keyframes = [Keyframe(0.0, {1: 500}), Keyframe(1.0, {1: 520})]
    plan = plan_playback(keyframes, {1: 400}, {1: 150}, now=0.0, time_scale=0.5, min_duration=0.3)
    first = plan.segments[0]
    assert first.start[1] == 400 and first.goal[1] == 500
    assert plan.segments[1].duration == pytest.approx(2.0)  # recorded 1 s at half speed
    with pytest.raises(ValueError):
        plan_playback(keyframes, {1: 400}, {1: 150}, now=0.0, time_scale=1.5, min_duration=0.3)


def test_plan_respects_acceleration_so_the_limiter_never_reshapes_it() -> None:
    plan = plan_move({1: 400, 2: 600}, {1: 450, 2: 590}, {1: 150, 2: 150}, now=0.0, min_duration=0.3,
                     label="short", acceleration=540)
    assert plan.end >= (5.7735 * 50 / 540) ** 0.5 - 1e-3
    limiters = {i: RateLimiter(v, 150, 600, 300, 700) for i, v in {1: 400, 2: 600}.items()}
    t, worst = 0.0, 0.0
    while t < plan.end + 0.2:
        t += 0.01
        targets, _, _ = plan.sample(t)
        for i, limiter in limiters.items():
            limiter.set_target(targets[i])
            worst = max(worst, abs(limiter.step(0.01) - targets[i]))
    assert worst < 1.0, f"limiter deviated {worst:.2f} steps from the plan"
