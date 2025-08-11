#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class SpeedEstimatorNode(Node):
    def __init__(self):
        # ROS2 node başlatılır
        super().__init__('speed_estimator_node')

        # ---- Parametreler ----
        # Odometry verisi alınacak topic
        self.declare_parameter('odom_topic', '/odom')
        # twist alanı varsa (doğrudan hız ölçümü) kullanılsın mı?
        self.declare_parameter('use_twist_if_available', True)
        # Doğrusal hızın yayınlanacağı topic
        self.declare_parameter('out_v_topic', '/speed_linear')
        # Açısal hızın yayınlanacağı topic
        self.declare_parameter('out_w_topic', '/speed_angular')

        # Parametre değerlerini al
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_twist  = bool(self.get_parameter('use_twist_if_available').value)
        self.out_v = str(self.get_parameter('out_v_topic').value)
        self.out_w = str(self.get_parameter('out_w_topic').value)

        # Hız yayıncıları oluştur
        self.pub_v = self.create_publisher(Float32, self.out_v, 10)
        self.pub_w = self.create_publisher(Float32, self.out_w, 10)

        # Son odometry verisinin zamanı ve konumu
        self._last_odom_time: Optional[Time] = None
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_yaw: Optional[float] = None

        # Odometry verisini dinleyen subscriber
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

        self.get_logger().info("SpeedEstimatorNode up.")

    def on_odom(self, msg: Odometry):
        """
        Odometry mesajı geldiğinde çalışır.
        Doğrusal ve açısal hız hesaplar veya mevcut twist verisini kullanır.
        """
        # Mesajın zaman damgasını al
        now = Time.from_msg(msg.header.stamp)

        # Twist alanındaki doğrusal (vx, vy) ve açısal (wz) hız değerleri
        vx, vy, wz = msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z

        # Pozisyon bilgileri
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        # Oryantasyon (quaternion → yaw dönüşümü)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Hesaplanacak hız değişkenleri
        v_lin = None
        v_ang = None

        # Eğer twist alanı kullanılacaksa ve geçerliyse, doğrudan kullan
        if self.use_twist and all(map(math.isfinite, [vx, vy, wz])):
            v_lin = float(math.hypot(vx, vy))  # doğrusal hız
            v_ang = float(wz)                  # açısal hız

        # Önceki ölçüm varsa, konum değişiminden hız hesapla
        if self._last_odom_time is not None and self._last_x is not None:
            dt = (now - self._last_odom_time).nanoseconds / 1e9  # saniye cinsinden zaman farkı
            if dt > 1e-3:  # çok küçük değerleri yoksay
                ds = math.hypot(x - self._last_x, y - self._last_y)  # mesafe farkı
                dyaw = self._angle_wrap(yaw - (self._last_yaw or 0.0))  # yön farkı
                if v_lin is None:
                    v_lin = float(ds / dt)     # mesafe/zaman → doğrusal hız
                if v_ang is None:
                    v_ang = float(dyaw / dt)   # açı/zaman → açısal hız

        # Son ölçümleri sakla
        self._last_odom_time, self._last_x, self._last_y, self._last_yaw = now, x, y, yaw

        # Hesaplanan hızları yayınla
        if v_lin is not None:
            mv = Float32()
            mv.data = v_lin
            self.pub_v.publish(mv)
        if v_ang is not None:
            mw = Float32()
            mw.data = v_ang
            self.pub_w.publish(mw)

    # Açıyı [-pi, pi] aralığında normalize eder.
    @staticmethod
    def _angle_wrap(a: float) -> float:
        """
        Açıyı [-pi, pi] aralığında normalize eder.
        """
        while a > math.pi:
            a -= 2.0*math.pi
        while a < -math.pi:
            a += 2.0*math.pi
        return a

def main():
    rclpy.init()
    rclpy.spin(SpeedEstimatorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
