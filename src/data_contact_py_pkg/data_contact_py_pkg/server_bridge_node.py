#!/usr/bin/env python3
import json
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String, Float32, Bool

# HTTP üzerinden veri göndermek için
import threading, queue, time
import requests

class ServerBridgeNode(Node):
    def __init__(self):
        # ROS2 node başlatılır
        super().__init__('server_bridge_node')

        # ---- Parametreler ----
        # Sunucu URL'si (telemetri verilerinin gönderileceği yer)
        self.declare_parameter('server_url', 'http://localhost:9090')
        # Yayınlama hızı (Hz cinsinden)
        self.declare_parameter('publish_rate_hz', 5.0)
        # Harita verisi (JPEG base64 formatında)
        self.declare_parameter('map_topic', '/map_jpeg_b64')
        # Doğrusal hız
        self.declare_parameter('v_topic', '/speed_linear')
        # Açısal hız
        self.declare_parameter('w_topic', '/speed_angular')
        # Engel var/yok bilgisi
        self.declare_parameter('obs_flag_topic', '/obstacle_detected')
        # Engel mesafesi bilgisi
        self.declare_parameter('obs_dist_topic', '/obstacle_distance')
        # Pil yüzdesi
        self.declare_parameter('battery_topic', '/battery_status')
        # Robot kimliği
        self.declare_parameter('robot_id', 'robot_001')

        # Parametreleri değişkenlere yükle
        self.server_url = str(self.get_parameter('server_url').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.robot_id = str(self.get_parameter('robot_id').value)

        # ---- Son durum cache (gelen veriler burada tutulur) ----
        self.map_b64: Optional[str] = None
        self.v: Optional[float] = None
        self.w: Optional[float] = None
        self.obs_flag: Optional[bool] = None
        self.obs_dist: Optional[float] = None
        self.battery: Optional[float] = None

        # ---- ROS topic abonelikleri ----
        self.create_subscription(String,  str(self.get_parameter('map_topic').value),       self.on_map, 10)
        self.create_subscription(Float32, str(self.get_parameter('v_topic').value),         self.on_v,   10)
        self.create_subscription(Float32, str(self.get_parameter('w_topic').value),         self.on_w,   10)
        self.create_subscription(Bool,    str(self.get_parameter('obs_flag_topic').value),  self.on_obs, 10)
        self.create_subscription(Float32, str(self.get_parameter('obs_dist_topic').value),  self.on_dist,10)
        self.create_subscription(Float32, str(self.get_parameter('battery_topic').value),   self.on_batt,10)

        # ---- HTTP gönderim kuyruğu ve arka plan thread'i ----
        self._q: "queue.Queue[dict]" = queue.Queue(maxsize=100)  # Gönderilecek verilerin kuyruğu
        self._http_thread = threading.Thread(target=self._http_worker, daemon=True)  # HTTP gönderim işçisi
        self._http_thread.start()

        # ---- Gönderim zamanlayıcısı ----
        period = max(0.05, 1.0 / max(self.rate_hz, 0.1))  # En az 0.05 sn aralık
        self.create_timer(period, self._tick_send)

        self.get_logger().info(f"ServerBridgeNode up. url={self.server_url} rate={self.rate_hz}Hz id={self.robot_id}")

    # ---- Gelen topic verilerini saklayan callback fonksiyonlar ----
    def on_map(self, msg: String):   self.map_b64 = msg.data
    def on_v(self, msg: Float32):    self.v = float(msg.data)
    def on_w(self, msg: Float32):    self.w = float(msg.data)
    def on_obs(self, msg: Bool):     self.obs_flag = bool(msg.data)
    def on_dist(self, msg: Float32): self.obs_dist = float(msg.data)
    def on_batt(self, msg: Float32): self.battery = float(msg.data)

    # ---- Zamanlayıcı: veriyi paketle ve kuyruğa ekle ----
    def _tick_send(self):
        now = self.get_clock().now().to_msg()
        payload = {
            "robot_id": self.robot_id,
            "timestamp": {  # ROS zaman damgası
                "sec": now.sec,
                "nanosec": now.nanosec
            },
            "telemetry": {  # Robotun son telemetri verileri
                "speed_linear": self.v,
                "speed_angular": self.w,
                "obstacle_detected": self.obs_flag,
                "obstacle_distance": self.obs_dist,
                "battery_percent": self.battery
            },
            # Harita verisi büyük olduğu için ayrı alan
            "map_jpeg_b64": self.map_b64
        }
        try:
            self._q.put_nowait(payload)  # Kuyruğa ekle
        except queue.Full:
            self.get_logger().warn("HTTP queue full, dropping telemetry.")

    # ---- HTTP gönderim işçisi ----
    def _http_worker(self):
        session = requests.Session()
        while True:
            data = self._q.get()  # Kuyruktan veri al
            if data is None:  # None gelirse thread sonlanır
                break
            try:
                r = session.post(self.server_url, json=data, timeout=5.0)  # HTTP POST isteği gönder
                if r.status_code >= 400:
                    print(f"[ServerBridge] HTTP error {r.status_code}: {r.text[:200]}")
            except Exception as e:
                print(f"[ServerBridge] HTTP post failed: {e}")

def main():
    rclpy.init()
    node = ServerBridgeNode()
    try:
        rclpy.spin(node)  # Node’u çalıştır
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
