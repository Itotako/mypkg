# SPDX-FileCopyrightText: 2025 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.state_subscription = self.create_subscription(
            String, 'battery_state', self.state_callback, 10)
        self.level_subscription = self.create_subscription(
            Float32, 'battery_level', self.level_callback, 10)

    def state_callback(self, msg):
        self.get_logger().info(f'Received Battery State: {msg.data}')

    def level_callback(self, msg):
        if msg.data <= 20.0:
            self.get_logger().warn(f'Battery Low: {msg.data}% remaining!')
        else:
            self.get_logger().info(f'Battery Level OK: {msg.data}%')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = BatteryMonitor()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

