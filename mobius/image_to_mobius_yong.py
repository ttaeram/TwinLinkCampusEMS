import base64
import configparser
import json
import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
except Exception as e:
    raise RuntimeError("cv_bridge와 opencv-python이 필요합니다: sudo apt install ros-$ROS_DISTRO-cv-bridge && pip install opencv-python") from e

from mqtt_publish import crt_cnt, publish_cin_only
import paho.mqtt.client as mqtt


# --------------------------
# Mobius / oneM2M 설정
# --------------------------
INI_PATH = '/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini'

config = configparser.ConfigParser()
if not config.read(INI_PATH):
    raise FileNotFoundError(f"config.ini를 찾을 수 없습니다: {INI_PATH}")

IOTPLATFORM_IP = config['API']['IOTPLATFORM_IP']
IOTPLATFORM_MQTT_PORT = int(config['API']['IOTPLATFORM_MQTT_PORT'])

# CSE/AE/Container 경로
CSE_BASE = '/Mobius'
AE_NAME = 'CampusEMS'
CNT_CHAIN = ['Robots', 'Robot07', 'ImageRawData']
AE_ID = 'CAdmin'
CIN_URI = f"{CSE_BASE}/{AE_NAME}/" + "/".join(CNT_CHAIN)


# --------------------------
# ROS2 노드
# --------------------------
class ImageToMobiusNode(Node):
    def __init__(self):
        super().__init__('image_to_mobius')

        # QoS: Sensor Data (Best Effort, volatile, depth=1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_name = '/s3/robot'
        self.bridge = CvBridge()

        # 전송 주기 제한(Hz). 0이면 모든 프레임 전송
        self.publish_hz = float(os.environ.get('PUB_HZ', '2.0'))
        self.min_interval = 0.0 if self.publish_hz <= 0 else 1.0 / self.publish_hz
        self.last_pub_ts = 0.0

        self.sub = self.create_subscription(Image, topic_name, self.cb_image, qos)
        self.get_logger().info(f"Subscribed: {topic_name} (BEST_EFFORT)")

    def cb_image(self, msg: Image):
        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        if self.min_interval and (now - self.last_pub_ts) < self.min_interval:
            return
        self.last_pub_ts = now

        # ROS Image -> OpenCV BGR
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        # JPEG 인코딩
        ok, enc = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().error("JPEG 인코딩 실패")
            return

        b64 = base64.b64encode(enc.tobytes()).decode('ascii')

        # oneM2M cin con에 넣을 JSON
        payload = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "encoding": "jpeg",
            "data": b64
        }

        try:
            # 질문에 제공된 함수 재사용: con에 JSON 문자열을 넣어 cin 생성
            publish_cin_only(CIN_URI, AE_ID, payload)
            self.get_logger().info(f"cin -> {CIN_URI} (size={len(b64)} base64 chars)")
        except Exception as e:
            self.get_logger().error(f"Mobius cin publish 실패: {e}")


def main():
    rclpy.init()
    node = ImageToMobiusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
