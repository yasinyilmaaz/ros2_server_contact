# random_pub.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class RandomPub(Node):
    def __init__(self):
        super().__init__('random_pub')
        self.pub = self.create_publisher(Float32, '/random_number', 10)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = Float32()
        msg.data = float(random.random()*100.0)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RandomPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
