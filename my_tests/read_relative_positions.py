import serial
import time
import json

PORT = "COM3"
BAUDRATE = 1000000

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

PRESENT_POSITION_ADDR = 56
READ_LENGTH = 2

OFFSET_FILE = "servo_offsets.json"


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

    print("Reading relative positions from offset...")
    print("Press Ctrl + C to stop.\n")

    try:
        while True:
            relative_values = {}

            for servo_id in SERVO_IDS:
                current_pos = read_position(ser, servo_id)
                offset = offsets.get(str(servo_id))

                if current_pos is not None and offset is not None:
                    relative_values[servo_id] = current_pos - offset
                else:
                    relative_values[servo_id] = None

            print(relative_values)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()