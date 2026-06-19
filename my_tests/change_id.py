import serial
import time

print("change_id.py started")

PORT = "COM3"
BAUDRATE = 1000000

OLD_ID = 1
NEW_ID = 7

ID_ADDRESS = 5
LOCK_ADDRESS = 55  # 0x37, STS3215 Lock flag


def checksum(packet_body):
    return (~sum(packet_body)) & 0xFF


def make_ping_packet(servo_id):
    body = [servo_id, 0x02, 0x01]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def make_write_packet(servo_id, address, data):
    body = [servo_id, 3 + len(data), 0x03, address] + data
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def ping(ser, servo_id):
    ser.reset_input_buffer()
    ser.write(make_ping_packet(servo_id))
    time.sleep(0.1)
    return ser.read(20)


def write_byte(ser, servo_id, address, value):
    packet = make_write_packet(servo_id, address, [value])
    print(f"Write addr {address}, value {value}: {packet.hex(' ')}")
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.2)
    return ser.read(20)


def main():
    print("Connect only ONE servo.")
    print(f"Changing ID {OLD_ID} -> {NEW_ID} with EEPROM unlock")

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)
    time.sleep(0.3)

    before = ping(ser, OLD_ID)
    print(f"Ping OLD ID {OLD_ID} before:", before.hex(" "))

    if not before:
        print(f"No response from ID {OLD_ID}. Stop.")
        ser.close()
        return

    # 1. Unlock EEPROM
    unlock_resp = write_byte(ser, OLD_ID, LOCK_ADDRESS, 0)
    print("Unlock response:", unlock_resp.hex(" "))

    # 2. Change ID
    id_resp = write_byte(ser, OLD_ID, ID_ADDRESS, NEW_ID)
    print("ID write response:", id_resp.hex(" "))

    time.sleep(0.5)

    # 3. Check new ID
    new_check = ping(ser, NEW_ID)
    print(f"Ping NEW ID {NEW_ID} after:", new_check.hex(" "))

    if not new_check:
        print("ID change not confirmed.")
        ser.close()
        return

    # 4. Lock EEPROM again using new ID
    lock_resp = write_byte(ser, NEW_ID, LOCK_ADDRESS, 1)
    print("Lock response:", lock_resp.hex(" "))

    ser.close()

    print(f"SUCCESS: Servo ID changed and saved as {NEW_ID}")
    print("Now power cycle the servo and scan again to confirm.")


if __name__ == "__main__":
    main()