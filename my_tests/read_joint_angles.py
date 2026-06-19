import serial
import time
import json
import math

PORT = "COM3"
BAUDRATE = 1000000

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

PRESENT_POSITION_ADDR = 56
READ_LENGTH = 2

OFFSET_FILE = "servo_offsets.json"

SIGNS = {
    1: 1,
    2: 1,
    3: -1,
    4: 1,
    5: 1,
    6: -1,
    7: 1,
}

JOINT_NAMES = {
    1: "Base",
    2: "Shoulder",
    3: "Elbow",
    4: "Wrist 1",
    5: "Wrist 2",
    6: "Wrist 3",
    7: "Gripper",
}


def checksum(packet_body):
    return (~sum(packet_body)) & 0xFF


def make_read_packet(servo_id, address, read_length):
    instruction = 0x02
    length = 0x04
    body = [servo_id, length, instruction, address, read_length]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def read_position(ser, servo_id):
    packet = make_read_packet(servo_id, PRESENT_POSITION_ADDR, READ_LENGTH)

    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.015)

    response = ser.read(20)

    if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
        low = response[5]
        high = response[6]
        position = low + (high << 8)
        return position

    return None


def main():
    with open(OFFSET_FILE, "r") as file:
        offsets = json.load(file)

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    print("Reading joint angles...")
    print("Press Ctrl + C to stop.\n")

    try:
        while True:
            for servo_id in SERVO_IDS:
                current_pos = read_position(ser, servo_id)
                offset = offsets.get(str(servo_id))

                if current_pos is not None and offset is not None:
                    relative = current_pos - offset
                    corrected = relative * SIGNS[servo_id]

                    degree = corrected * 360 / 4096
                    radian = corrected * 2 * math.pi / 4096

                    name = JOINT_NAMES[servo_id]
                    print(f"{name:10s}: {degree:8.2f} deg | {radian:8.3f} rad")
                else:
                    name = JOINT_NAMES[servo_id]
                    print(f"{name:10s}: read failed")

            print("-" * 45)
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()