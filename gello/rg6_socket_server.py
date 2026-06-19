import socket
import time
import threading

from scservo_sdk import PortHandler
from scservo_sdk.packet_handler import PacketHandler
from scservo_sdk.packet_handler import COMM_SUCCESS


# ============================================================
# UR / Pendant socket settings
# ============================================================
HOST = "0.0.0.0"
PORT = 5005


# ============================================================
# Servo settings
# ============================================================
SERVO_PORT = "COM3"
SERVO_BAUDRATE = 1000000
GRIPPER_SERVO_ID = 7


# ============================================================
# STS3215 address
# Present Position address is commonly 56 for STS/SCS servos
# ============================================================
ADDR_PRESENT_POSITION = 56


# ============================================================
# Calibration
# 실행하면 raw 값이 뜸.
# 닫힌 자세 raw 값 / 열린 자세 raw 값을 보고 나중에 수정.
# ============================================================
SERVO_RAW_CLOSE = 3800
SERVO_RAW_OPEN = 3400


# ============================================================
# RG6 width range
# 우선 안전하게 10~100mm
# ============================================================
RG6_WIDTH_CLOSE = 10.0
RG6_WIDTH_OPEN = 100.0


# 방향 반대면 True로 변경
REVERSE_DIRECTION = True


# 부드러움 정도
SMOOTHING_ALPHA = 0.7
SEND_HZ = 15.0
SERVO_READ_HZ = 30.0


running = True
target_width = RG6_WIDTH_OPEN
current_width = RG6_WIDTH_OPEN


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))


def make_packet_handler():
    """
    scservo_sdk 버전에 따라 PacketHandler() 또는 PacketHandler(0)가 다를 수 있어서
    둘 다 시도.
    """
    try:
        return PacketHandler(0)
    except TypeError:
        return PacketHandler()


def map_raw_servo_to_rg6_width(raw_position):
    if SERVO_RAW_OPEN == SERVO_RAW_CLOSE:
        return RG6_WIDTH_CLOSE

    ratio = (float(raw_position) - SERVO_RAW_CLOSE) / (
        SERVO_RAW_OPEN - SERVO_RAW_CLOSE
    )

    ratio = clamp(ratio, 0.0, 1.0)

    if REVERSE_DIRECTION:
        ratio = 1.0 - ratio

    width = RG6_WIDTH_CLOSE + ratio * (RG6_WIDTH_OPEN - RG6_WIDTH_CLOSE)
    return clamp(width, RG6_WIDTH_CLOSE, RG6_WIDTH_OPEN)


def open_servo_bus():
    port_handler = PortHandler(SERVO_PORT)
    packet_handler = make_packet_handler()

    if not port_handler.openPort():
        raise RuntimeError(f"Failed to open servo port: {SERVO_PORT}")

    if not port_handler.setBaudRate(SERVO_BAUDRATE):
        raise RuntimeError(f"Failed to set baudrate: {SERVO_BAUDRATE}")

    print(f"[SERVO] Opened {SERVO_PORT} at {SERVO_BAUDRATE}")
    return port_handler, packet_handler


def read_servo_position(packet_handler, port_handler, servo_id):
    """
    STS3215 현재 위치 raw value 읽기.
    """

    result = packet_handler.read2ByteTxRx(
        port_handler,
        servo_id,
        ADDR_PRESENT_POSITION,
    )

    # 보통 반환값:
    # (value, comm_result, error)
    if isinstance(result, tuple):
        position = result[0]
        comm_result = result[1] if len(result) > 1 else COMM_SUCCESS
        error = result[2] if len(result) > 2 else 0
    else:
        position = result
        comm_result = COMM_SUCCESS
        error = 0

    if comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Communication error: {comm_result}")

    if error != 0:
        raise RuntimeError(f"Servo error: {error}")

    return int(position)


def servo_reader_loop():
    global running, target_width

    try:
        port_handler, packet_handler = open_servo_bus()
    except Exception as e:
        print("[SERVO] Failed to open servo bus:", e)
        running = False
        return

    print(f"[SERVO] Reading gripper servo ID {GRIPPER_SERVO_ID}")
    print("[SERVO] Move the gripper servo and watch raw values.")
    print("[SERVO] Press Ctrl+C to stop.")
    print("")

    try:
        while running:
            try:
                raw = read_servo_position(
                    packet_handler,
                    port_handler,
                    GRIPPER_SERVO_ID,
                )

                target_width = map_raw_servo_to_rg6_width(raw)

                print(
                    f"[SERVO] raw={raw:4d} -> target_width={target_width:5.1f} mm",
                    end="\r",
                )

                time.sleep(1.0 / SERVO_READ_HZ)

            except Exception as e:
                print("\n[SERVO] Read error:", e)
                time.sleep(0.5)

    finally:
        try:
            port_handler.closePort()
            print("\n[SERVO] Port closed.")
        except Exception:
            pass


def gripper_socket_server():
    global running, current_width

    while running:
        print(f"\n[RG6] Waiting for UR connection on port {PORT}...")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind((HOST, PORT))
            server.listen(1)

            conn, addr = server.accept()
            print(f"[RG6] UR connected from {addr}")

            try:
                while running:
                    current_width = (
                        current_width * (1.0 - SMOOTHING_ALPHA)
                        + target_width * SMOOTHING_ALPHA
                    )

                    # UR socket_read_ascii_float expects this format: (55.0)
                    msg = f"({current_width:.1f})\n"
                    conn.sendall(msg.encode("utf-8"))

                    time.sleep(1.0 / SEND_HZ)

            except Exception as e:
                print("\n[RG6] Connection closed:", e)

            finally:
                try:
                    conn.close()
                except Exception:
                    pass

        except Exception as e:
            print("\n[RG6] Server error:", e)

        finally:
            try:
                server.close()
            except Exception:
                pass

            time.sleep(1.0)


if __name__ == "__main__":
    print("")
    print("RG6 servo-follow socket server")
    print("--------------------------------")
    print(f"Servo port       : {SERVO_PORT}")
    print(f"Servo baudrate   : {SERVO_BAUDRATE}")
    print(f"Gripper servo ID : {GRIPPER_SERVO_ID}")
    print(f"Raw close/open   : {SERVO_RAW_CLOSE} / {SERVO_RAW_OPEN}")
    print(f"RG6 close/open   : {RG6_WIDTH_CLOSE} / {RG6_WIDTH_OPEN} mm")
    print(f"Reverse direction: {REVERSE_DIRECTION}")
    print("")

    servo_thread = threading.Thread(target=servo_reader_loop, daemon=True)
    socket_thread = threading.Thread(target=gripper_socket_server, daemon=True)

    servo_thread.start()
    socket_thread.start()

    try:
        while running:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping...")
        running = False
        time.sleep(1.0)