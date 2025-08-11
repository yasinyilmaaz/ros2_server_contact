#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float32

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parametreler tanımlanır (ROS parametre server üzerinden değiştirilebilir)
        self.declare_parameter('scan_topic', '/scan')  # LIDAR/LaserScan veri konusu
        self.declare_parameter('range_topic', '/range')  # Ultrasonik veya benzeri Range sensör konusu
        self.declare_parameter('range_threshold', 0.2)  # Engel algılama mesafesi (metre)
        self.declare_parameter('out_flag_topic', '/obstacle_detected')  # Engel var/yok bilgisi yayın konusu
        self.declare_parameter('out_dist_topic', '/obstacle_distance')  # Engel mesafesi yayın konusu

        # Parametre değerleri okunur
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.range_topic = str(self.get_parameter('range_topic').value)
        self.thr = float(self.get_parameter('range_threshold').value)
        self.out_flag = str(self.get_parameter('out_flag_topic').value)
        self.out_dist = str(self.get_parameter('out_dist_topic').value)

        # Başlangıçta minimum mesafe sonsuz olarak ayarlanır
        self._min_range = float('inf')

        # LIDAR verisini dinleyen abonelik
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        # Ultrasonik sensör verisini dinleyen abonelik
        self.create_subscription(Range, self.range_topic, self.on_range, 10)

        # Engel var/yok bilgisini yayınlayan publisher
        self.pub_flag = self.create_publisher(Bool, self.out_flag, 10)
        # Engel mesafesini yayınlayan publisher
        self.pub_dist = self.create_publisher(Float32, self.out_dist, 10)

        self.get_logger().info(f"ObstacleDetectorNode up. thr={self.thr}m")

    def on_scan(self, msg: LaserScan):
        """
        LIDAR/LaserScan verisi geldiğinde çalışır.
        Mesafeler filtrelenerek minimum değer bulunur.
        """
        # LIDAR'ın ölçebildiği min ve max mesafe değerleri
        rmin = msg.range_min if math.isfinite(msg.range_min) else 0.0
        rmax = msg.range_max if math.isfinite(msg.range_max) else float('inf')

        # Geçerli (NaN veya sonsuz olmayan) mesafeleri filtrele
        vals = [r for r in msg.ranges if math.isfinite(r) and rmin <= r <= rmax]

        # Minimum mesafe bulunur (hiç ölçüm yoksa sonsuz)
        self._apply_range(min(vals) if vals else float('inf'))

    def on_range(self, msg: Range):
        """
        Ultrasonik veya tek nokta mesafe sensörü verisi geldiğinde çalışır.
        Mesafe geçerli aralıktaysa değerlendirilir.
        """
        r = msg.range if math.isfinite(msg.range) else float('inf')
        rmin = msg.min_range if math.isfinite(msg.min_range) else 0.0
        rmax = msg.max_range if math.isfinite(msg.max_range) else float('inf')

        # Eğer ölçüm sensör aralığı dışındaysa görmezden gel
        if not (rmin <= r <= rmax):
            r = float('inf')

        self._apply_range(r)

    def _apply_range(self, r: float):
        """
        Minimum mesafeyi günceller ve engel bilgilerini yayınlar.
        """
        self._min_range = float(r)

        # Engel var mı? (mesafe threshold'dan küçükse True)
        flag = (self._min_range < self.thr)

        # Boolean engel var/yok mesajı
        mo = Bool()
        mo.data = bool(flag)

        # Engel mesafesi mesajı
        md = Float32()
        md.data = float(self._min_range if math.isfinite(self._min_range) else float('inf'))

        # Mesajları yayınla
        self.pub_flag.publish(mo)
        self.pub_dist.publish(md)

def main():
    # ROS2 başlatılır
    rclpy.init()

    # Node çalıştırılır
    rclpy.spin(ObstacleDetectorNode())

    # Kapatılır
    rclpy.shutdown()

if __name__ == '__main__':
    main()
