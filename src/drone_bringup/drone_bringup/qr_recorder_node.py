"""Save /qr/item locally without sending data to a server."""
import json
import sqlite3
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from drone_bringup.qr_record_store import QrRecordStore


class QrRecorderNode(Node):
    def __init__(self):
        super().__init__('qr_recorder_node')
        self.declare_parameter('output_path', '')
        self.declare_parameter('duplicate_window_sec', 3.0)
        output = self.get_parameter('output_path').value
        if not output:
            raise ValueError('Set output_path to a local database file')
        path = Path(output).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._store = QrRecordStore(
            str(path), self.get_parameter('duplicate_window_sec').value)
        self._sub = self.create_subscription(String, '/qr/item', self._on_item, 10)
        self.get_logger().info(f'Local QR database: {path}')

    def _on_item(self, msg):
        try:
            record = self._store.record(json.loads(msg.data))
        except (ValueError, TypeError, sqlite3.Error) as exc:
            self.get_logger().error(f'QR record not saved: {exc}')
            return
        if record is not None:
            self.get_logger().info(f'Saved QR observation: {record["client_scan_id"]}')

    def destroy_node(self):
        self._store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QrRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
