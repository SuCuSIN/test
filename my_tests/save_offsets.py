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
    print("Set the controller to the neutral/reference pose.")
    input("Press Enter when the physical model is in the reference pose...")

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    offsets = {}

    print("\nReading offset values...")

    for servo_id in SERVO_IDS:
        readings = []

        for _ in range(10):
            pos = read_position(ser, servo_id)
            if pos is not None:
                readings.append(pos)
            time.sleep(0.03)

        if readings:
            avg_pos = round(sum(readings) / len(readings))
            offsets[str(servo_id)] = avg_pos
            print(f"ID {servo_id}: offset = {avg_pos}")
        else:
            offsets[str(servo_id)] = None
            print(f"ID {servo_id}: failed to read")

    ser.close()

    with open(OFFSET_FILE, "w") as file:
        json.dump(offsets, file, indent=4)

    print(f"\nOffsets saved to {OFFSET_FILE}")


if __name__ == "__main__":
    main()