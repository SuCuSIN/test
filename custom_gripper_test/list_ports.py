"""List serial ports visible to pyserial."""

from serial.tools import list_ports


def main() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}: {port.description}")


if __name__ == "__main__":
    main()

