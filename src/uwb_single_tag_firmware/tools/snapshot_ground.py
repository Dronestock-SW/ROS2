"""Read ROS ground state and current tag output without opening either USB port."""
import argparse
import collections
import datetime
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State, ExtendedState
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = {'started_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'counts': collections.Counter(), 'heartbeats': [], 'ground': {}}
    rclpy.init()
    node = rclpy.create_node('esp32_ground_snapshot')
    def state(msg):
        result['ground'].update(connected=msg.connected, armed=msg.armed, mode=msg.mode,
                               state_received_monotonic_s=time.monotonic())
    def extended(msg):
        result['ground'].update(landed_state=msg.landed_state,
                               extended_received_monotonic_s=time.monotonic())
    with args.output.with_suffix('.jsonl').open('w', encoding='utf-8') as stream:
        def received(msg):
            try:
                envelope = json.loads(msg.data)
                value = envelope.get('message', envelope)
                result['counts'][value.get('type', 'unknown')] += 1
                if value.get('type') == 'uwb_raw_status':
                    result['heartbeats'].append(value)
                stream.write(json.dumps({'received_monotonic_s': time.monotonic(),
                                         'envelope': envelope}, ensure_ascii=False)+'\n')
            except (ValueError, TypeError, AttributeError):
                result['counts']['parse_error'] += 1
        node.create_subscription(State, '/mavros/state', state, qos_profile_sensor_data)
        node.create_subscription(ExtendedState, '/mavros/extended_state', extended, qos_profile_sensor_data)
        node.create_subscription(String, '/uwb/received', received, qos_profile_sensor_data)
        end = time.monotonic()+args.seconds
        try:
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    result['finished_monotonic_s'] = time.monotonic()
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
