"""`dexhand-servo tune-servo`: guarded changes to a servo's position-loop P gain and minimum start force.

The SCS0009 position loop is proportional-only (output ~ P x error). On joints with high
friction it needs a large error before the joint moves, which shows up as stick-slip,
lag and a residual error with the load left on. Raising P (register 21) gives more force
per step of error; minimum start force (register 24) adds a fixed force once the error
leaves the dead band, aimed at static friction. Both are EEPROM registers.

Safety:
- only registers 21, 24 and the EEPROM lock (48) can be written, enforced at packet level
  by `TuningPolicy`, with value ranges (P 5..60, start force 0..100 raw);
- the servo must have torque off; the operator types the servo ID to confirm;
- the values found before the first ever change are saved once as `id<K>_original.json`
  under data/servo_tuning/, every change is also logged, and `--restore` writes the
  originals back; everything is read back and verified.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from .control.storage import DATA_DIR
from .registers import REG_LOCK, REG_TORQUE_ENABLE
from .safe_bus import SafeBus, UnsafeOperationError, WritePolicy

REG_P_GAIN = 21
REG_D_GAIN = 22
REG_MIN_START = 24
P_GAIN_RANGE = (5, 60)
MIN_START_RANGE = (0, 100)  # raw units; the SCSCL table lists 0.1 % per unit -> 0..10 %
LARGE_P_STEP = 10
TUNING_DIR = DATA_DIR / "servo_tuning"


class TuningAborted(RuntimeError):
    """A precondition failed or the operator declined; nothing was written."""


class TuningPolicy(WritePolicy):
    def __init__(self) -> None:
        super().__init__("servo-tuning", frozenset({REG_P_GAIN, REG_MIN_START, REG_LOCK}))

    def check_write(self, packet: Sequence[int]) -> None:
        super().check_write(packet)  # single byte, allowed address, lock value 0/1
        address, value = packet[5], packet[6]
        ranges = {REG_P_GAIN: P_GAIN_RANGE, REG_MIN_START: MIN_START_RANGE}
        if address in ranges:
            lo, hi = ranges[address]
            if not lo <= value <= hi:
                raise UnsafeOperationError(f"register {address} value {value} outside {lo}..{hi}")


TUNING = TuningPolicy()


@dataclass(frozen=True)
class TuningValues:
    p_gain: int
    d_gain: int
    min_start: int


@dataclass
class TuneResult:
    servo_id: int
    before: TuningValues
    after: TuningValues | None
    verified: bool
    steps: list[str]


def read_tuning(bus: SafeBus, servo_id: int) -> tuple[TuningValues, int] | None:
    """(P, D, minimum start force) and the torque-enable state, or None without a clean reply."""
    gains = bus.read(servo_id, REG_P_GAIN, REG_MIN_START - REG_P_GAIN + 1)
    torque = bus.read(servo_id, REG_TORQUE_ENABLE, 1)
    if not (gains.ok and torque.ok and len(gains.data) == 4 and torque.data):
        return None
    return TuningValues(gains.data[0], gains.data[1], gains.data[3]), torque.data[0]


def _original_path(directory: Path, servo_id: int) -> Path:
    return directory / f"id{servo_id}_original.json"


def _save_backup(directory: Path, servo_id: int, name: str, values: TuningValues, change: dict,
                 action: str = "tune") -> None:
    directory.mkdir(parents=True, exist_ok=True)
    record = {"servo_id": servo_id, "joint": name, "time": datetime.now().isoformat(timespec="seconds"),
              **asdict(values)}
    original = _original_path(directory, servo_id)
    if not original.exists():
        original.write_text(json.dumps(record, indent=2))
    with (directory / "changes.jsonl").open("a") as log:
        log.write(json.dumps({**record, "action": action, "change": change}) + "\n")


def load_original(directory: Path, servo_id: int) -> TuningValues:
    path = _original_path(directory, servo_id)
    if not path.exists():
        raise TuningAborted(f"no saved original values for ID {servo_id} ({path})")
    data = json.loads(path.read_text())
    return TuningValues(int(data["p_gain"]), int(data["d_gain"]), int(data["min_start"]))


def tune_servo(
    bus: SafeBus,
    servo_id: int,
    name: str,
    confirm: Callable[[str], bool],
    p_gain: int | None = None,
    min_start: int | None = None,
    directory: Path = TUNING_DIR,
) -> TuneResult:
    if p_gain is None and min_start is None:
        raise TuningAborted("nothing to change (give --p-gain and/or --min-start, or --restore)")
    for label, value, (lo, hi) in (("P gain", p_gain, P_GAIN_RANGE), ("min start force", min_start, MIN_START_RANGE)):
        if value is not None and not lo <= value <= hi:
            raise TuningAborted(f"{label} {value} outside the allowed range {lo}..{hi}")
    state = read_tuning(bus, servo_id)
    if state is None:
        raise TuningAborted(f"ID {servo_id} does not reply cleanly")
    before, torque = state
    if torque:
        raise TuningAborted(f"ID {servo_id} has torque on; switch it off (GUI 'Torque off', or close the GUI) first")

    target = TuningValues(p_gain if p_gain is not None else before.p_gain, before.d_gain,
                          min_start if min_start is not None else before.min_start)
    if target == before:
        raise TuningAborted("requested values equal the current ones")
    summary = (f"ID {servo_id} ({name}): P gain {before.p_gain} -> {target.p_gain}, "
               f"min start force {before.min_start} -> {target.min_start} (D gain {before.d_gain} unchanged)")
    if target.p_gain - before.p_gain > LARGE_P_STEP:
        summary += f"\nWARNING: P rises by more than {LARGE_P_STEP} at once; larger steps risk oscillation."
    if not confirm(summary):
        raise TuningAborted("operator did not confirm")

    _save_backup(directory, servo_id, name, before,
                 {"p_gain": target.p_gain, "min_start": target.min_start})
    return _write_values(bus, servo_id, before, target)


def restore_servo(
    bus: SafeBus, servo_id: int, name: str, confirm: Callable[[str], bool], directory: Path = TUNING_DIR
) -> TuneResult:
    original = load_original(directory, servo_id)
    state = read_tuning(bus, servo_id)
    if state is None:
        raise TuningAborted(f"ID {servo_id} does not reply cleanly")
    before, torque = state
    if torque:
        raise TuningAborted(f"ID {servo_id} has torque on; switch it off first")
    if not confirm(f"ID {servo_id} ({name}): restore P gain {before.p_gain} -> {original.p_gain}, "
                   f"min start force {before.min_start} -> {original.min_start}"):
        raise TuningAborted("operator did not confirm")
    _save_backup(directory, servo_id, name, before,
                 {"p_gain": original.p_gain, "min_start": original.min_start}, action="restore")
    return _write_values(bus, servo_id, before, original)


def _write_values(bus: SafeBus, servo_id: int, before: TuningValues, target: TuningValues) -> TuneResult:
    steps: list[str] = []
    with bus.use_policy(TUNING):
        unlock = bus.write_byte(servo_id, REG_LOCK, 0)
        steps.append(f"unlock EEPROM: {unlock.result.value}")
        if unlock.ok:
            if target.p_gain != before.p_gain:
                steps.append(f"write P gain {target.p_gain}: {bus.write_byte(servo_id, REG_P_GAIN, target.p_gain).result.value}")
            if target.min_start != before.min_start:
                written = bus.write_byte(servo_id, REG_MIN_START, target.min_start)
                steps.append(f"write min start force {target.min_start}: {written.result.value}")
        lock = bus.write_byte(servo_id, REG_LOCK, 1)
        steps.append(f"re-lock EEPROM: {lock.result.value}")
    state = read_tuning(bus, servo_id)
    after = state[0] if state else None
    verified = after == target and unlock.ok and lock.ok
    steps.append(f"read back: {after}" + ("" if verified else "  <- does NOT match the request"))
    return TuneResult(servo_id, before, after, verified, steps)
