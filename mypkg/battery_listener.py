# SPDX-FileCopyrightText: 2025 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')
        self.subscription = self.create_subscription(
            String,
            'battery_state',
            self.listener_callback,
            10  # キューサイズ
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

