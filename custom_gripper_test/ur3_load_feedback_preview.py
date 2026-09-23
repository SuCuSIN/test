"""Shadow-mode load feedback with AnySkin gating. No actuator writes."""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from load_feedback_signal import LoadFeedbackSignal
from ur3_load_probe import sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.50.14')
    parser.add_argument('--contact-topic', default='/gello/gripper_contact_hold')
    args = parser.parse_args()
    import rclpy
    from std_msgs.msg import Bool, Float32
    from rtde_receive import RTDEReceiveInterface

    rclpy.init()
    node = rclpy.create_node('ur3_load_feedback_preview')
    publisher = node.create_publisher(Float32, '/gello/load_feedback_preview', 1)
    signal = LoadFeedbackSignal()
    contact = [False, None]

    def on_contact(msg):
        contact[:] = [bool(msg.data), time.monotonic()]

    subscription = node.create_subscription(Bool, args.contact_topic, on_contact, 1)
    folder = Path(__file__).parent / 'load_reports'
    folder.mkdir(exist_ok=True)
    path = folder / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_preview.jsonl')
    receiver = None
    print(f'SHADOW ONLY: no torque or payload changes. Log: {path}', flush=True)
    print('Empty stationary reference required; active AnySkin contact heartbeat required.', flush=True)
    try:
        receiver = RTDEReceiveInterface(args.robot_ip, 50.0)
        last_stamp = None
        last_fresh = time.monotonic()
        last_log = 0.0
        with path.open('w', encoding='utf-8') as file:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                now = time.monotonic()
                age = now - contact[1] if contact[1] is not None else 1e9
                row = sample(receiver) if receiver.isConnected() else None
                if row is not None and row['robot_s'] != last_stamp:
                    if last_stamp is not None and row['robot_s'] < last_stamp:
                        signal.invalidate('robot clock reset')
                    last_stamp = row['robot_s']
                    last_fresh = now
                    signal.update(now, [row[k] for k in ('fx', 'fy', 'fz')],
                                  [row[f'q{i + 1}'] for i in range(6)],
                                  row['stationary'], contact[0], age)
                if now - last_fresh > 0.5 or age > 0.5:
                    signal.invalidate('feedback stale')
                publisher.publish(Float32(data=signal.level))
                if now - last_log >= 0.5:
                    record = {'pc_monotonic_s': now, 'preview_level': signal.level,
                              'state': signal.state, 'contact': contact[0],
                              'force_sample': row, 'actuation_enabled': False}
                    file.write(json.dumps(record) + '\n')
                    file.flush()
                    print(f'preview={signal.level:.2f}; {signal.state}', flush=True)
                    last_log = now
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            publisher.publish(Float32(data=0.0))
        if receiver is not None:
            receiver.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
