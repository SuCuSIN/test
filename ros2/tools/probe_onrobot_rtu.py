#!/usr/bin/env python3
import argparse
import struct
import sys
import time

import serial


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def with_crc(body: bytes) -> bytes:
    crc = crc16_modbus(body)
    return body + struct.pack("<H", crc)


def make_read_holding_request(slave_id: int, address: int, count: int) -> bytes:
    return with_crc(struct.pack(">BBHH", slave_id, 0x03, address, count))


def make_write_single_request(slave_id: int, address: int, value: int) -> bytes:
    return with_crc(struct.pack(">BBHH", slave_id, 0x06, address, value))


def make_write_multiple_request(slave_id: int, address: int, values: list[int]) -> bytes:
    body = struct.pack(">BBHHB", slave_id, 0x10, address, len(values), 2 * len(values))
    body += b"".join(struct.pack(">H", value & 0xFFFF) for value in values)
    return with_crc(body)


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe OnRobot RG Modbus RTU over /tmp/ttyUR.")
    parser.add_argument("--device", default="/tmp/ttyUR")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--parity", choices=["N", "E", "O"], default="E")
    parser.add_argument("--stopbits", choices=["1", "2"], default="1")
    parser.add_argument("--slave", type=int, default=65)
    parser.add_argument("--address", type=int, default=275, help="275 = width with offset")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--write-single", type=int, default=None)
    parser.add_argument("--write-multiple", type=int, nargs="*", default=None)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if args.write_single is not None:
        request = make_write_single_request(args.slave, args.address, args.write_single)
        mode = f"write single address={args.address} value={args.write_single}"
    elif args.write_multiple is not None:
        request = make_write_multiple_request(args.slave, args.address, args.write_multiple)
        mode = f"write multiple address={args.address} values={args.write_multiple}"
    else:
        request = make_read_holding_request(args.slave, args.address, args.count)
        mode = f"read holding address={args.address} count={args.count}"

    print(mode)
    parity = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }[args.parity]
    stopbits = serial.STOPBITS_TWO if args.stopbits == "2" else serial.STOPBITS_ONE

    print(
        f"Opening {args.device} baud={args.baud} "
        f"parity={args.parity} stopbits={args.stopbits} timeout={args.timeout}"
    )
    print("TX:", request.hex(" "))

    try:
        with serial.Serial(
            args.device,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=parity,
            stopbits=stopbits,
            timeout=args.timeout,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(request)
            ser.flush()
            time.sleep(0.05)
            response = ser.read(256)
    except Exception as exc:
        print(f"ERROR opening/sending: {exc}", file=sys.stderr)
        return 2

    if not response:
        print("RX: <timeout/no response>")
        return 1

    print("RX:", response.hex(" "))
    if len(response) >= 5:
        payload, received_crc_bytes = response[:-2], response[-2:]
        expected_crc = crc16_modbus(payload)
        received_crc = struct.unpack("<H", received_crc_bytes)[0]
        print(f"CRC expected=0x{expected_crc:04x} received=0x{received_crc:04x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
