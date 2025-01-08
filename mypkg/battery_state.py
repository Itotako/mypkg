# SPDX-FileCopyrightText: 2024 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import subprocess

class BatteryStatePublisher(Node):
    def __init__(self):
        super().__init__('battery_state_publisher')
        # 充電状態とバッテリー残量をそれぞれ別々に送る
        self.state_publisher_ = self.create_publisher(String, 'battery_state', 10)
        self.level_publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Battery State Publisher has been started!')

    def timer_callback(self):
        # バッテリー状態を取得
        result = subprocess.run(['acpi'], capture_output=True, text=True)
        self.get_logger().info(f'ACPI output: {result.stdout.strip()}')

        # 充電状態の判定
        state = 'Unknown'
        battery_level = 0.0

        if 'Charging' in result.stdout:
            state = 'Charging'
        elif 'Discharging' in result.stdout:
            state = 'Not Charging'

        # 残量を取得
        if '%' in result.stdout:
            percentage_start = result.stdout.find('%') - 2
            percentage = result.stdout[percentage_start:percentage_start + 3].strip()
            try:
                battery_level = float(percentage.replace('%', ''))
                if battery_level <= 20:
                    self.get_logger().warn(f'Low Battery Warning: {battery_level}% remaining!')
            except ValueError:
                self.get_logger().error('Failed to parse battery percentage.')

        # 充電状態とバッテリー残量を別々に発信
        state_msg = String()
        state_msg.data = state
        self.state_publisher_.publish(state_msg)
        self.get_logger().info(f'Publishing State: {state_msg.data}')

        level_msg = Float32()
        level_msg.data = battery_level
        self.level_publisher_.publish(level_msg)
        self.get_logger().info(f'Publishing Battery Level: {level_msg.data}%')

def main(args=None):
    rclpy.init(args=args)

    battery_state_publisher = BatteryStatePublisher()

    try:
        rclpy.spin(battery_state_publisher)
    except KeyboardInterrupt:
        pass  # Ctrl+Cで例外を無視してプログラムを終了させる
    finally:
        battery_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

