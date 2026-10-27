import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'recibido: {msg.data}')


def main():
    rclpy.init()
    rclpy.spin(Listener())
