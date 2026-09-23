"""Read GELLO lever endpoints without sending torque or motion commands."""

import argparse
import statistics
import time
import sys
from pathlib import Path

try:
    from scservo_sdk import PacketHandler, PortHandler
except ImportError:
    # Match the publisher's existing local SDK fallback.
    local_packages = Path(__file__).resolve().parents[2] / '.venv' / 'Lib' / 'site-packages'
    if not (local_packages / 'scservo_sdk').is_dir():
        raise
    sys.path.append(str(local_packages))
    from scservo_sdk import PacketHandler, PortHandler


def capture(port, packet, servo_id, label):
    input(f"Hold lever comfortably at {label}, then press Enter: ")
    values = []
    for _ in range(40):
        value, result, error = packet.read2ByteTxRx(port, servo_id, 56)
        if result == 0 and error == 0 and 0 <= value <= 4095:
            values.append(value)
        time.sleep(0.025)
    if len(values) < 30:
        raise RuntimeError(f"Insufficient valid reads: {len(values)}/40")
    if max(values) - min(values) > 25:
        raise RuntimeError("Lever readings unstable or cross encoder wrap; do not apply these endpoints")
    value = round(statistics.median(values))
    print(f"{label}: median={value}, min={min(values)}, max={max(values)}")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--id', type=int, default=7)
    args = parser.parse_args()
    port = PortHandler(args.port)
    try:
        if not port.openPort() or not port.setBaudRate(1000000):
            raise RuntimeError('Cannot open controller serial port')
        packet = PacketHandler(0)
        print('Read-only. Stop ROS/control programs first. Do not force a powered lever.')
        opened = capture(port, packet, args.id, 'OPEN')
        closed = capture(port, packet, args.id, 'CLOSED')
        if abs(opened - closed) < 50:
            raise RuntimeError('Endpoint span too small; verify lever travel and servo ID')
        print(f'Controller endpoints: OPEN={opened}, CLOSED={closed}')
        print('No settings changed. Gripper motor endpoints remain unchanged.')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
