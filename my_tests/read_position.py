import serial
import time

PORT = "COM3"
BAUDRATE = 1000000
SERVO_ID = 1

# STS/SCS 계열 위치값 주소
PRESENT_POSITION_ADDR = 56
READ_LENGTH = 2

def checksum(packet_body):
    return (~sum(packet_body)) & 0xFF

def make_read_packet(servo_id, address, read_length):
    # FF FF ID LENGTH INSTRUCTION ADDRESS READ_LENGTH CHECKSUM
    instruction = 0x02  # READ
    length = 0x04
    body = [servo_id, length, instruction, address, read_length]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def read_position():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    packet = make_read_packet(SERVO_ID, PRESENT_POSITION_ADDR, READ_LENGTH)

    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.02)

    response = ser.read(20)
    ser.close()

    print("Raw response:", response.hex(" "))

    if len(response) >= 7:
        # expected: FF FF ID LENGTH ERROR PARAM_L PARAM_H CHECKSUM
        low = response[5]
        high = response[6]
        position = low + (high << 8)

        print(f"Servo ID {SERVO_ID} position: {position}")
    else:
        print("No valid position response.")

if __name__ == "__main__":
    read_position()