import time
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead

PORT = "COM3"
BAUDRATE = 1000000

# STS3215 Present Position
ADDR_PRESENT_POSITION = 56
LEN_PRESENT_POSITION = 2

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

# STS / SCServo protocol end
PROTOCOL_END = 0


def main():
    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(PROTOCOL_END)

    if not port_handler.openPort():
        print("Failed to open port")
        return

    if not port_handler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        return

    group_sync_read = GroupSyncRead(
        port_handler,
        packet_handler,
        ADDR_PRESENT_POSITION,
        LEN_PRESENT_POSITION,
    )

    for servo_id in SERVO_IDS:
        ok = group_sync_read.addParam(servo_id)
        if not ok:
            print(f"Failed to add servo ID {servo_id}")

    print("Sync read test started. Press Ctrl+C to stop.")

    try:
        while True:
            result = group_sync_read.txRxPacket()

            positions = {}

            for servo_id in SERVO_IDS:
                available = group_sync_read.isAvailable(
                    servo_id,
                    ADDR_PRESENT_POSITION,
                    LEN_PRESENT_POSITION,
                )

                if available:
                    pos = group_sync_read.getData(
                        servo_id,
                        ADDR_PRESENT_POSITION,
                        LEN_PRESENT_POSITION,
                    )
                    positions[servo_id] = pos
                else:
                    positions[servo_id] = None

            print(positions)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        group_sync_read.clearParam()
        port_handler.closePort()


if __name__ == "__main__":
    main()