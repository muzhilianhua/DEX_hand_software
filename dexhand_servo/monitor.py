"""Read-only continuous sampling of the present-state block (position, speed, load, voltage, temperature).

Two uses: characterise feedback noise (e.g. a servo whose position jumps with
torque off) and measure a joint's mechanical range by moving it by hand while
min/max are recorded. Only READ packets are sent.
"""

from __future__ import annotations

import csv
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from itertools import pairwise
from pathlib import Path

from .registers import PRESENT_BLOCK_ADDR, PRESENT_BLOCK_LEN, decode
from .safe_bus import SafeBus

# SCS0009 nominal: 300 degrees over 0..1023 (datasheet value; verify per joint if precision matters).
DEG_PER_STEP = 300.0 / 1024

SPARK_CHARS = "▁▂▃▄▅▆▇█"


@dataclass
class Sample:
    t: float  # seconds since monitor start
    result: str
    position: int | None = None
    speed: int | None = None
    load: float | None = None
    voltage: float | None = None
    temperature: int | None = None


@dataclass
class SeriesStats:
    servo_id: int
    samples: int
    ok: int
    failed: int
    position_min: int | None
    position_max: int | None
    position_spread: int | None
    position_mean: float | None
    position_std: float | None
    max_step: int | None  # largest change between consecutive clean samples
    jumps: int  # consecutive changes larger than the jump threshold
    voltage_min: float | None
    voltage_max: float | None
    temperature_max: int | None
    last_position: int | None

    @property
    def spread_deg(self) -> float | None:
        return None if self.position_spread is None else round(self.position_spread * DEG_PER_STEP, 1)

    def to_dict(self) -> dict[str, object]:
        return {**self.__dict__, "spread_deg": self.spread_deg}


@dataclass
class MonitorResult:
    started_at: str
    port: str
    baudrate: int
    ids: tuple[int, ...]
    rate_hz: float
    duration_s: float
    series: dict[int, list[Sample]] = field(default_factory=dict)
    elapsed_s: float = 0.0
    cycles: int = 0
    overruns: int = 0
    interrupted: bool = False

    @property
    def achieved_rate_hz(self) -> float:
        return self.cycles / self.elapsed_s if self.elapsed_s else 0.0


def compute_stats(servo_id: int, samples: list[Sample], jump_threshold: int) -> SeriesStats:
    clean = [s for s in samples if s.position is not None]
    positions = [s.position for s in clean]
    steps = [abs(b - a) for a, b in pairwise(positions)]
    voltages = [s.voltage for s in clean if s.voltage is not None]
    temperatures = [s.temperature for s in clean if s.temperature is not None]
    return SeriesStats(
        servo_id=servo_id,
        samples=len(samples),
        ok=len(clean),
        failed=len(samples) - len(clean),
        position_min=min(positions) if positions else None,
        position_max=max(positions) if positions else None,
        position_spread=max(positions) - min(positions) if positions else None,
        position_mean=round(statistics.fmean(positions), 2) if positions else None,
        position_std=round(statistics.pstdev(positions), 2) if len(positions) > 1 else None,
        max_step=max(steps) if steps else None,
        jumps=sum(step > jump_threshold for step in steps),
        voltage_min=min(voltages) if voltages else None,
        voltage_max=max(voltages) if voltages else None,
        temperature_max=max(temperatures) if temperatures else None,
        last_position=positions[-1] if positions else None,
    )


def sparkline(values: list[int], width: int = 40) -> str:
    values = values[-width:]
    if not values:
        return ""
    lo, hi = min(values), max(values)
    if hi == lo:
        return SPARK_CHARS[0] * len(values)
    scale = (len(SPARK_CHARS) - 1) / (hi - lo)
    return "".join(SPARK_CHARS[round((v - lo) * scale)] for v in values)


def run_monitor(
    bus: SafeBus,
    ids: tuple[int, ...],
    rate_hz: float,
    duration_s: float,
    on_cycle: Callable[[MonitorResult], None] | None = None,
) -> MonitorResult:
    """Sample every ID once per cycle until `duration_s` elapses (0 = until Ctrl-C).

    Ctrl-C ends the run normally: the partial result is returned so it can still be saved.
    """
    result = MonitorResult(
        started_at=datetime.now().isoformat(timespec="seconds"),
        port=bus.port_name,
        baudrate=bus.baudrate,
        ids=ids,
        rate_hz=rate_hz,
        duration_s=duration_s,
        series={servo_id: [] for servo_id in ids},
    )
    period = 1.0 / rate_hz
    keep_history = bus.keep_history
    bus.keep_history = False
    start = time.perf_counter()
    next_cycle = start
    try:
        while True:
            for servo_id in ids:
                t = bus.read(servo_id, PRESENT_BLOCK_ADDR, PRESENT_BLOCK_LEN)
                sample = Sample(t=round(time.perf_counter() - start, 5), result=t.result.value)
                if t.ok and len(t.data) == PRESENT_BLOCK_LEN:
                    d = decode(t.data, base_addr=PRESENT_BLOCK_ADDR)
                    sample.position = int(d["present_position"].value)
                    sample.speed = int(d["present_speed"].value)
                    sample.load = float(d["present_load"].value)
                    sample.voltage = float(d["present_voltage"].value)
                    sample.temperature = int(d["present_temperature"].value)
                result.series[servo_id].append(sample)
            result.cycles += 1
            result.elapsed_s = time.perf_counter() - start
            if on_cycle:
                on_cycle(result)
            if duration_s and result.elapsed_s >= duration_s:
                break
            next_cycle += period
            delay = next_cycle - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif -delay > period:
                # Bus too slow for the requested rate: resync instead of bursting to catch up.
                result.overruns += 1
                next_cycle = time.perf_counter()
    except KeyboardInterrupt:
        result.interrupted = True
    finally:
        bus.keep_history = keep_history
        result.elapsed_s = time.perf_counter() - start
    return result


def write_csv(result: MonitorResult, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = sorted(
        ((s.t, servo_id, s) for servo_id, samples in result.series.items() for s in samples),
        key=lambda row: (row[0], row[1]),
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "id", "result", "position", "speed", "load_pct", "voltage_v", "temperature_c"])
        for t, servo_id, s in rows:
            writer.writerow([t, servo_id, s.result, s.position, s.speed, s.load, s.voltage, s.temperature])
    return path

