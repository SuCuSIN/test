import serial
import time

PORT = "COM3"
BAUDRATE = 1000000


def checksum(packet_body):
    return (~sum(packet_body)) & 0xFF


def make_ping_packet(servo_id):
    body = [servo_id, 0x02, 0x01]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def find_valid_packet(data, expected_id):
    # Look for FF FF ID LENGTH ERROR CHECKSUM
    for i in range(len(data) - 5):
        if data[i] == 0xFF and data[i + 1] == 0xFF:
            packet_id = data[i + 2]
            length = data[i + 3]

            if packet_id == expected_id and length >= 2:
                packet_end = i + 4 + length
                if packet_end <= len(data):
                    return data[i:packet_end]

    return None


def scan():
    print(f"Scanning {PORT} at {BAUDRATE} baud...")

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)
    time.sleep(0.1)

    found = []

    for servo_id in range(1, 8):
        ser.reset_input_buffer()
        time.sleep(0.02)

        packet = make_ping_packet(servo_id)
        ser.write(packet)
        time.sleep(0.1)

        response = ser.read(64)

        if response:
            print(f"ID {servo_id} raw response: {response.hex(' ')}")

        valid_packet = find_valid_packet(response, servo_id)

        if valid_packet:
            print(f"Valid response from ID {servo_id}: {valid_packet.hex(' ')}")
            found.append(servo_id)

    ser.close()

    print("\nFound valid servo IDs:", found)


if __name__ == "__main__":
    scan()