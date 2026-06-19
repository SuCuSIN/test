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

# 네가 확인한 방향값: ++-++-+
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

# STS3215 position count를 radian으로 변환
COUNT_TO_RAD = 2 * math.pi / 4096


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


def wrapped_delta(current, offset):
    """
    0~4095 값이 경계를 넘어갈 때 갑자기 큰 값으로 튀는 문제를 줄이는 함수.
    예: 4090에서 10으로 넘어가도 작은 움직임으로 계산.
    """
    return (current - offset + 2048) % 4096 - 2048


def position_to_radian(servo_id, current_pos, offset):
    delta = wrapped_delta(current_pos, offset)
    corrected = delta * SIGNS[servo_id]
    radian = corrected * COUNT_TO_RAD
    return radian


def main():
    with open(OFFSET_FILE, "r") as file:
        offsets = json.load(file)

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    print("Reading UR5e-like joint values...")
    print("Press Ctrl + C to stop.\n")

    try:
        while True:
            joint_radians = []
            gripper_value = None

            for servo_id in SERVO_IDS:
                current_pos = read_position(ser, servo_id)
                offset = offsets.get(str(servo_id))

                if current_pos is None or offset is None:
                    print(f"ID {servo_id} read failed")
                    continue

                radian = position_to_radian(servo_id, current_pos, offset)

                if servo_id <= 6:
                    joint_radians.append(radian)
                else:
                    gripper_value = radian

            if len(joint_radians) == 6:
                print("UR5e joints rad:")
                print([round(value, 4) for value in joint_radians])

                print("Gripper:")
                print(round(gripper_value, 4) if gripper_value is not None else None)

                print("-" * 50)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()