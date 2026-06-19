import serial
import time
import csv

PORT = "COM3"
BAUDRATE = 1000000

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

PRESENT_POSITION_ADDR = 56
READ_LENGTH = 2

RECORD_SECONDS = 20   # 기록 시간, 필요하면 60으로 바꿔도 됨
CSV_FILE = "servo_positions.csv"


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
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    start_time = time.time()

    with open(CSV_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)

        header = ["time"] + [f"ID_{sid}" for sid in SERVO_IDS]
        writer.writerow(header)

        print("Recording started...")
        print("Move the controller now.")

        try:
            while True:
                current_time = time.time() - start_time

                if current_time > RECORD_SECONDS:
                    break

                row = [current_time]

                for servo_id in SERVO_IDS:
                    pos = read_position(ser, servo_id)
                    row.append(pos)

                writer.writerow(row)
                print(row)

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("Recording stopped by user.")

    ser.close()
    print(f"Saved data to {CSV_FILE}")


if __name__ == "__main__":
    main()