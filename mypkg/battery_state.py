# SPDX-FileCopyrightText: 2025 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import subprocess

class BatteryStatePublisher(Node):
    def __init__(self, mock_acpi_output=None):
        super().__init__('battery_state_publisher')
        self.state_publisher_ = self.create_publisher(String, 'battery_state', 10)
        self.level_publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.mock_acpi_output = mock_acpi_output
        self.get_logger().info('Battery State Publisher started!')

    def get_acpi_output(self):
        if self.mock_acpi_output is not None:
            return self.mock_acpi_output
        result = subprocess.run(['acpi'], capture_output=True, text=True)
        return result.stdout

    def timer_callback(self):
        acpi_output = self.get_acpi_output()
        self.get_logger().info(f'ACPI output: {acpi_output.strip()}')

        state = 'Unknown'
        battery_level = 0.0

        if 'Charging' in acpi_output:
            state = 'Charging'
        elif 'Discharging' in acpi_output:
            state = 'Not Charging'

        if '%' in acpi_output:
            percentage_start = acpi_output.find('%') - 2
            percentage = acpi_output[percentage_start:percentage_start + 3].strip()
            try:
                battery_level = float(percentage.replace('%', ''))
                if battery_level <= 20:
                    self.get_logger().warn(f'Low Battery Warning: {battery_level}% remaining!')
            except ValueError:
                self.get_logger().error('Failed to parse battery percentage.')

        state_msg = String()
        state_msg.data = state
        self.state_publisher_.publish(state_msg)
        self.get_logger().info(f'Publishing State: {state_msg.data}')

        level_msg = Float32()
        level_msg.data = battery_level
        self.level_publisher_.publish(level_msg)
        self.get_logger().info(f'Publishing Battery Level: {level_msg.data}%')

