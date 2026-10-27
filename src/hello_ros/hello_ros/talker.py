import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.n = 0
        self.create_timer(0.5, self.tick)

    def tick(self):
        msg = String(data=f'hola ROS 2 #{self.n}')
        self.n += 1
        self.pub.publish(msg)
        self.get_logger().info(f'publicado: {msg.data}')


def main():
    rclpy.init()
    rclpy.spin(Talker())
