import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.subscription = self.create_subscription(
            String, 'pressed_keys', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info(f"Received keys: {msg.data}")
        else:
            self.get_logger().info("No keys are pressed")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

