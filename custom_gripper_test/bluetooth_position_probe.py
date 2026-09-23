"""Read ESP32 replies without issuing motion commands."""

import argparse
import socket
import time
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=15555)
    args = parser.parse_args()
    replies = ''
    with socket.create_connection((args.host, args.port), timeout=3) as connection:
        connection.settimeout(0.25)
        for command in ('PING', 'READ 1', 'READ 2'):
            print('>', command, flush=True)
            connection.sendall((command + '\n').encode('ascii'))
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                try:
                    data = connection.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError('Bridge disconnected')
                text = data.decode('utf-8', errors='replace')
                replies += text
                print(text, end='', flush=True)
    found = {int(value) for value in re.findall(r'POSITION id=(\d+).*?raw=-?\d+', replies)}
    missing = {1, 2} - found
    if missing:
        raise RuntimeError(f'No valid POSITION response for servo IDs {sorted(missing)}')
    print('Both servo positions received. Ready to start tracking.')


if __name__ == '__main__':
    main()
