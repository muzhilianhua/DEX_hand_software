"""SCS0009 (SCSCL family) memory map and decoding.

Addresses follow the SCSCL memory table (third-party transcription) and the
SDK's scscl.py constants. Treat decoded values as indicative until checked
against the official SCS0009 manual; the raw 0..70 dump is always kept so the
interpretation can be redone later. 16-bit values are big-endian (high byte at
the lower address) on potentiometer servos; register 2 (END) reports this.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

DUMP_END = 71  # registers 0..70

REG_END_FLAG = 2
REG_ID = 5
REG_BAUD = 6
REG_TORQUE_ENABLE = 40
REG_LOCK = 48
REG_PRESENT_POSITION = 56

PRESENT_BLOCK_ADDR = 56
PRESENT_BLOCK_LEN = 8  # position, speed, load, voltage, temperature

MAX_SERVO_ID = 253

BAUD_CODES: dict[int, int] = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 76_800,
    6: 57_600,
    7: 38_400,
}
SUPPORTED_BAUDRATES = frozenset(BAUD_CODES.values())


@dataclass(frozen=True)
class Register:
    addr: int
    size: int
    name: str
    area: str  # "EEPROM" | "SRAM"
    access: str  # "R" | "RW"
    unit: str = ""
    scale: float = 1.0
    sign_bit: int | None = None  # sign-magnitude encoding, as in the SDK's scs_tohost


REGISTERS: tuple[Register, ...] = (
    Register(0, 1, "firmware_major", "EEPROM", "R"),
    Register(1, 1, "firmware_minor", "EEPROM", "R"),
    Register(2, 1, "end_flag", "EEPROM", "R"),
    Register(3, 2, "model_number", "EEPROM", "R"),
    Register(5, 1, "id", "EEPROM", "RW"),
    Register(6, 1, "baud_code", "EEPROM", "RW"),
    Register(8, 1, "response_level", "EEPROM", "RW"),
    Register(9, 2, "min_angle_limit", "EEPROM", "RW", "step"),
    Register(11, 2, "max_angle_limit", "EEPROM", "RW", "step"),
    Register(13, 1, "max_temperature", "EEPROM", "RW", "degC"),
    Register(14, 1, "max_input_voltage", "EEPROM", "RW", "V", 0.1),
    Register(15, 1, "min_input_voltage", "EEPROM", "RW", "V", 0.1),
    Register(16, 2, "max_torque", "EEPROM", "RW", "%", 0.1),
    Register(18, 1, "phase", "EEPROM", "RW"),
    Register(19, 1, "unload_condition", "EEPROM", "RW"),
    Register(20, 1, "led_alarm_condition", "EEPROM", "RW"),
    Register(21, 1, "p_gain", "EEPROM", "RW"),
    Register(22, 1, "d_gain", "EEPROM", "RW"),
    Register(24, 1, "min_startup_force", "EEPROM", "RW", "%", 0.1),
    Register(26, 1, "cw_dead_zone", "EEPROM", "RW", "step"),
    Register(27, 1, "ccw_dead_zone", "EEPROM", "RW", "step"),
    Register(37, 1, "holding_torque", "EEPROM", "RW", "%"),
    Register(38, 1, "protection_time", "EEPROM", "RW", "ms", 10.0),
    Register(39, 1, "overload_threshold", "EEPROM", "RW", "%"),
    Register(40, 1, "torque_enable", "SRAM", "RW"),
    Register(42, 2, "goal_position", "SRAM", "RW", "step"),
    Register(44, 2, "goal_time", "SRAM", "RW", "ms"),
    Register(46, 2, "goal_speed", "SRAM", "RW", "step/s"),
    Register(48, 1, "lock", "SRAM", "RW"),
    Register(56, 2, "present_position", "SRAM", "R", "step"),
    Register(58, 2, "present_speed", "SRAM", "R", "step/s", sign_bit=15),
    Register(60, 2, "present_load", "SRAM", "R", "%", 0.1, sign_bit=10),
    Register(62, 1, "present_voltage", "SRAM", "R", "V", 0.1),
    Register(63, 1, "present_temperature", "SRAM", "R", "degC"),
    Register(64, 1, "async_write_flag", "SRAM", "R"),
    Register(65, 1, "status", "SRAM", "R"),
    Register(66, 1, "moving", "SRAM", "R"),
    Register(69, 2, "present_current", "SRAM", "R"),
)

REGISTERS_BY_NAME = {r.name: r for r in REGISTERS}


@dataclass(frozen=True)
class DecodedValue:
    addr: int
    raw: int
    value: float | int
    unit: str


def word_be(high: int, low: int) -> int:
    return ((high & 0xFF) << 8) | (low & 0xFF)


def _sign_magnitude(raw: int, sign_bit: int | None) -> int:
    if sign_bit is not None and raw & (1 << sign_bit):
        return -(raw & ~(1 << sign_bit))
    return raw


def decode_register(reg: Register, raw_bytes: Sequence[int]) -> DecodedValue:
    raw = raw_bytes[0] if reg.size == 1 else word_be(raw_bytes[0], raw_bytes[1])
    signed = _sign_magnitude(raw, reg.sign_bit)
    value: float | int = round(signed * reg.scale, 3) if reg.scale != 1.0 else signed
    return DecodedValue(reg.addr, raw, value, reg.unit)


def decode(memory: Sequence[int | None], base_addr: int = 0) -> dict[str, DecodedValue]:
    """Decode every known register fully covered by `memory` (None marks unread bytes)."""
    decoded: dict[str, DecodedValue] = {}
    for reg in REGISTERS:
        offset = reg.addr - base_addr
        if offset < 0 or offset + reg.size > len(memory):
            continue
        chunk = memory[offset : offset + reg.size]
        if any(b is None for b in chunk):
            continue
        decoded[reg.name] = decode_register(reg, chunk)  # type: ignore[arg-type]
    return decoded
