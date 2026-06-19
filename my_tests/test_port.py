import serial
import time

PORT = "COM3"
BAUDRATE = 1000000

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(1)

    print(f"Connected to {PORT} at {BAUDRATE} baud")
    print("Serial port opened successfully.")

    ser.close()

except Exception as e:
    print("Failed to open serial port.")
    print(e)