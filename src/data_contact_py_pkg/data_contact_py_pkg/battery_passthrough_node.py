#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryStatusNode(Node):
    """
    Dışarıdan veri almaz. 100.0'dan başlar,
    her saniye %0.1 azalır ve 0.0'da sabitlenir.
    Tek topic: /battery_status
    """
    def __init__(self):
        super().__init__('battery_status_node')

        # Yayıncı
        self.publisher_ = self.create_publisher(Float32, '/battery_status', 10)

        # Başlangıç değeri ve düşüş miktarı
        self.level = 100.0
        self.dec_per_tick = 0.1

        # 1 saniyede bir tetiklenecek timer
        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info("BatteryStatusNode up. Publishing simulated battery to /battery_status")

    def _tick(self):
        if self.level > 0.0:
            self.level = max(0.0, self.level - self.dec_per_tick)

        msg = Float32()
        msg.data = float(self.level)
        self.publisher_.publish(msg)

        self.get_logger().info(f'Battery: {self.level:.1f}%')

def main():
    rclpy.init()
    node = BatteryStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
