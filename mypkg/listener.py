import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.sub = create_subscription(Int16, "countup", cd, 10)

    def cd(self):
        self.get_logger().info("Listen: %d" % msg.data)

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
#    self.sub = create_subscription(Int16, "countup", cd, 10)

#rclpy.init()
#node = Node("listener")

#def cb(msg):
#    global node
#    node.get_logger().info("Listen: %d" % msg.data)

#def main():
#    sub = node.create_subscription(Int16, "countup", cb, 10)
#    rclpy.spin(node)
