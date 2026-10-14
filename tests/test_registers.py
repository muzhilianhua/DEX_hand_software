from __future__ import annotations

from dexhand_servo.registers import BAUD_CODES, DUMP_END, decode, word_be


def test_word_is_big_endian() -> None:
    assert word_be(0x01, 0xF4) == 500


def test_decode_present_block() -> None:
    # position 500, speed -10 (bit 15 sign), load -20 % raw 200 (bit 10 sign), 5.0 V, 31 degC
    block = [0x01, 0xF4, 0x80, 0x0A, 0x04, 0xC8, 50, 31]
    d = decode(block, base_addr=56)
    assert d["present_position"].value == 500
    assert d["present_speed"].value == -10
    assert d["present_load"].value == -20.0
    assert d["present_voltage"].value == 5.0
    assert d["present_temperature"].value == 31


def test_decode_skips_unread_bytes() -> None:
    memory: list[int | None] = [0] * DUMP_END
    memory[56] = None
    d = decode(memory)
    assert "present_position" not in d
    assert "present_voltage" in d


def test_baud_codes() -> None:
    assert BAUD_CODES[0] == 1_000_000
    assert len(BAUD_CODES) == 8
