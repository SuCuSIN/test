"""Minimal STS3215 protocol helpers used by the standalone gripper tests."""

from __future__ import annotations

import time
from typing import Optional

import serial


HEADER = b"\xff\xff"
INSTRUCTION_PING = 0x01
INSTRUCTION_READ = 0x02
INSTRUCTION_WRITE = 0x03
ID_ADDRESS = 5
LOCK_ADDRESS = 55
PRESENT_POSITION_ADDRESS = 56
GOAL_POSITION_ADDRESS = 41
MIN_SERVO_ID = 0
MAX_SERVO_ID = 253


def validate_servo_id(servo_id: int) -> None:
    if not MIN_SERVO_ID <= servo_id <= MAX_SERVO_ID:
        raise ValueError(f"Servo ID must be {MIN_SERVO_ID}..{MAX_SERVO_ID}: {servo_id}")


def _checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _packet(servo_id: int, instruction: int, parameters: bytes = b"") -> bytes:
    validate_servo_id(servo_id)
    body = bytes((servo_id, len(parameters) + 2, instruction)) + parameters
    return HEADER + body + bytes((_checksum(body),))


def open_bus(port: str, baudrate: int, timeout: float = 0.08) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def _read_status(ser: serial.Serial, expected_id: int) -> Optional[bytes]:
    """Read and minimally validate one STS status packet."""
    deadline = time.monotonic() + ser.timeout
    data = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            data.extend(chunk)
            start = data.find(HEADER)
            if start >= 0 and len(data) >= start + 4:
                length = data[start + 3]
                packet_length = length + 4
                if len(data) >= start + packet_length:
                    packet = bytes(data[start : start + packet_length])
                    body = packet[2:-1]
                    if packet[2] == expected_id and packet[-1] == _checksum(body):
                        return packet
                    return None
    return None


def transact(ser: serial.Serial, packet: bytes, expected_id: int) -> Optional[bytes]:
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    return _read_status(ser, expected_id)


def ping(ser: serial.Serial, servo_id: int) -> bool:
    return transact(ser, _packet(servo_id, INSTRUCTION_PING), servo_id) is not None


def status_error(packet: Optional[bytes]) -> Optional[int]:
    if packet is None or len(packet) < 6:
        return None
    return packet[4]


def status_parameters(packet: Optional[bytes]) -> bytes:
    if packet is None or len(packet) < 6:
        return b""
    return packet[5:-1]


def read_bytes(ser: serial.Serial, servo_id: int, address: int, length: int) -> Optional[bytes]:
    if not 0 <= address <= 255:
        raise ValueError(f"Address must be 0..255: {address}")
    if not 1 <= length <= 255:
        raise ValueError(f"Read length must be 1..255: {length}")
    packet = transact(
        ser,
        _packet(servo_id, INSTRUCTION_READ, bytes((address, length))),
        servo_id,
    )
    parameters = status_parameters(packet)
    if len(parameters) != length:
        return None
    return parameters


def read_word(ser: serial.Serial, servo_id: int, address: int) -> Optional[int]:
    data = read_bytes(ser, servo_id, address, 2)
    if data is None:
        return None
    return data[0] | (data[1] << 8)


def read_position(ser: serial.Serial, servo_id: int) -> Optional[int]:
    return read_word(ser, servo_id, PRESENT_POSITION_ADDRESS)


def write_byte(ser: serial.Serial, servo_id: int, address: int, value: int) -> bool:
    if not 0 <= value <= 255:
        raise ValueError(f"Byte value must be 0..255: {value}")
    parameters = bytes((address, value))
    return transact(ser, _packet(servo_id, INSTRUCTION_WRITE, parameters), servo_id) is not None


def write_position(
    ser: serial.Serial,
    servo_id: int,
    position: int,
    speed: int = 1000,
    acceleration: int = 50,
) -> bool:
    if not 0 <= position <= 4095:
        raise ValueError(f"Position must be 0..4095: {position}")
    if not 0 <= speed <= 4095:
        raise ValueError(f"Speed must be 0..4095: {speed}")
    if not 0 <= acceleration <= 255:
        raise ValueError(f"Acceleration must be 0..255: {acceleration}")

    parameters = bytes(
        (
            GOAL_POSITION_ADDRESS,
            acceleration,
            position & 0xFF,
            (position >> 8) & 0xFF,
            0,
            0,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
        )
    )
    return transact(ser, _packet(servo_id, INSTRUCTION_WRITE, parameters), servo_id) is not None
