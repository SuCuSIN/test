import socket
import threading
import unittest
from unittest.mock import Mock

from esp32_bluetooth.windows_bluetooth_tcp_bridge import serial_to_client
from bluetooth_anyskin_gripper_shell import BluetoothGripperClient


class BridgeTest(unittest.TestCase):
    def test_contact_reads_sent_together_and_matched_by_id(self):
        client = BluetoothGripperClient('unused', 0)
        client.position_callback = Mock()
        client.lines.put('POSITION id=1 logical=-47 raw=2000 status=0x0')

        def reply(command):
            client.lines.put('POSITION id=2 logical=153 raw=2200 status=0x0')
            client.lines.put('POSITION id=1 logical=-247 raw=1800 status=0x0')

        client.send = Mock(side_effect=reply)
        self.assertEqual(client.read_contact_positions(), (1800, 2200))
        client.send.assert_called_once_with('READ 1\nREAD 2')
        client.position_callback.assert_not_called()

    def test_contact_batch_error_is_not_ignored(self):
        client = BluetoothGripperClient('unused', 0)
        client.send = Mock(side_effect=lambda command: client.lines.put('ERROR: servo not found: ID 2'))
        with self.assertRaisesRegex(RuntimeError, 'servo not found'):
            client.read_contact_positions()
        client.send.assert_called_once()

    def test_contact_batch_requires_both_servos_without_retry(self):
        client = BluetoothGripperClient('unused', 0)
        client.send = Mock(side_effect=lambda command: client.lines.put(
            'POSITION id=1 logical=-247 raw=1800 status=0x0'))
        with self.assertRaisesRegex(TimeoutError, 'batch incomplete'):
            client.read_contact_positions(timeout=0.01)
        client.send.assert_called_once()

    def test_move_send_failure_never_reconnects_or_replays(self):
        client = BluetoothGripperClient('unused', 0)
        sock = Mock()
        sock.sendall.side_effect = OSError('broken pipe')
        client.sock = sock
        client.connect = Mock()
        with self.assertRaisesRegex(ConnectionError, 'not reconnected or replayed'):
            client.send('SYNC_MOVE 1 -47 2 -47 120 22')
        sock.sendall.assert_called_once()
        client.connect.assert_not_called()
        self.assertIsNone(client.sock)

    def test_ack_timeout_includes_command_and_connection_status(self):
        client = BluetoothGripperClient('unused', 0)
        client.last_command = 'SYNC_MOVE 1 -47 2 -47 20 1'
        with self.assertRaisesRegex(TimeoutError, 'command=.*20 1.*tcp_connected=False'):
            client.wait_for_command_ack('SYNC_MOVE sent', timeout=0)

    def test_serial_failure_closes_tcp_and_worker_exits(self):
        servo = Mock()
        servo.in_waiting = 24
        servo.read.side_effect = [b'POSITION id=1 raw=2000\n', OSError('serial disconnected')]
        stopped = threading.Event()
        sender, receiver = socket.socketpair()
        with sender, receiver:
            receiver.settimeout(1)
            worker = threading.Thread(target=serial_to_client, args=(servo, sender, stopped))
            worker.start()
            received = b''
            while True:
                chunk = receiver.recv(1024)
                if not chunk:
                    break
                received += chunk
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
            self.assertTrue(stopped.is_set())
            self.assertEqual(received, b'POSITION id=1 raw=2000\n')

    def test_partial_response_is_forwarded_without_waiting_for_newline(self):
        servo = Mock()
        servo.in_waiting = 3
        servo.read.side_effect = [b'ACK', OSError('done')]
        client = Mock()
        serial_to_client(servo, client, threading.Event())
        client.sendall.assert_called_once_with(b'ACK')
        servo.read.assert_called_with(3)

    def test_empty_buffer_waits_for_one_byte(self):
        servo = Mock()
        servo.in_waiting = 0
        servo.read.side_effect = OSError('done')
        serial_to_client(servo, Mock(), threading.Event())
        servo.read.assert_called_once_with(1)


if __name__ == '__main__':
    unittest.main()
