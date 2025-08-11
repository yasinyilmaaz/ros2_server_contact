#!/usr/bin/env python3
import base64
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class MapToJpegNode(Node):
    def __init__(self):
        super().__init__('map_to_jpeg_node')

        # ---- Parametreler ----
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 640)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/map_jpeg_b64')
        self.declare_parameter('publish_compressed_image', True)
        self.declare_parameter('compressed_topic', '/map_jpeg/compressed')
        self.declare_parameter('flip_mode', 'none')

        self.W = int(self.get_parameter('image_width').value)
        self.H = int(self.get_parameter('image_height').value)
        self.Q = int(self.get_parameter('jpeg_quality').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.pub_compr_enabled = bool(self.get_parameter('publish_compressed_image').value)
        self.compressed_topic = str(self.get_parameter('compressed_topic').value)
        self.flip_mode = str(self.get_parameter('flip_mode').value).lower()

        # QoS: /map genelde latched
        qos_map = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)

        # IO
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos_map)
        self.pub_map_b64 = self.create_publisher(String, self.output_topic, 10)
        self.pub_compressed = self.create_publisher(CompressedImage, self.compressed_topic, 10) if self.pub_compr_enabled else None

        self.get_logger().info(f"MapToJpegNode up ({self.W}x{self.H}) flip={self.flip_mode}")

    def on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0 or len(msg.data) != w*h:
            self.get_logger().warn("Invalid /map.")
            return

        # grid -> görüntü
        arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 127, np.uint8)  # bilinmeyen: gri
        known = (arr >= 0)
        img[known] = (100 - arr[known]) * 255 // 100  # 0: beyaz, 100: siyah
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # flip uygula
        img_color = self._apply_flip_to_image(img_color)

        # encode & publish
        out_img = cv2.resize(img_color, (self.W, self.H), interpolation=cv2.INTER_NEAREST)
        ok, enc = cv2.imencode('.jpg', out_img, [cv2.IMWRITE_JPEG_QUALITY, self.Q])
        if not ok:
            self.get_logger().error("JPEG encode failed")
            return

        # Base64 encode
        b64 = base64.b64encode(enc.tobytes()).decode('ascii')
        s = String()
        s.data = b64
        self.pub_map_b64.publish(s)

        # görseli yayınla
        if self.pub_compressed:
            cim = CompressedImage()
            cim.format = 'jpeg'
            cim.data = enc.tobytes()
            self.pub_compressed.publish(cim)

    def _apply_flip_to_image(self, img):
        if self.flip_mode == 'x':
            return cv2.flip(img, 1)
        if self.flip_mode == 'y':
            return cv2.flip(img, 0)
        if self.flip_mode == 'xy':
            return cv2.flip(img, -1)
        return img

def main():
    rclpy.init()
    node = MapToJpegNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
