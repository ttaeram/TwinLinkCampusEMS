import base64
import configparser
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
    raise RuntimeError(
        "cv_bridge와 opencv-python이 필요합니다: "
        "sudo apt install ros-$ROS_DISTRO-cv-bridge && pip install opencv-python"
    ) from e

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

CSE_BASE = '/Mobius'
AE_NAME = 'CampusEMS'
AE_ID   = 'CAdmin'
CIN_URI_CAM1 = f"{CSE_BASE}/{AE_NAME}/CCTV/YongdukHall/CCTV01/ImageRawData"
CIN_URI_CAM2 = f"{CSE_BASE}/{AE_NAME}/CCTV/YongdukHall/CCTV02/ImageRawData"


# --------------------------
# ROS2 노드
# --------------------------
class ImageToMobiusNode(Node):
    def __init__(self):
        super().__init__('image_to_mobius_multi_cam')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        # 전송 주기 제한(Hz). 0이면 모든 프레임 전송
        self.publish_hz = float(os.environ.get('PUB_HZ', '2.0'))
        self.min_interval = 0.0 if self.publish_hz <= 0 else 1.0 / self.publish_hz

        # 카메라별 마지막 전송 시각 (초 단위)
        self.last_pub_ts = {
            "cam1": 0.0,
            "cam2": 0.0,
        }

        # 토픽 이름
        self.topic_cam1 = '/s3/cam1'
        self.topic_cam2 = '/s3/cam2'

        # 구독 등록
        self.sub_cam1 = self.create_subscription(
            Image, self.topic_cam1, self.cb_image_cam1, qos
        )
        self.sub_cam2 = self.create_subscription(
            Image, self.topic_cam2, self.cb_image_cam2, qos
        )

        self.get_logger().info(f"Subscribed: {self.topic_cam1} (CCTV01, BEST_EFFORT)")
        self.get_logger().info(f"Subscribed: {self.topic_cam2} (CCTV02, BEST_EFFORT)")
        self.get_logger().info(f"CIN_URI_CAM1 = {CIN_URI_CAM1}")
        self.get_logger().info(f"CIN_URI_CAM2 = {CIN_URI_CAM2}")

    # -------- 카메라별 콜백 -------- #

    def cb_image_cam1(self, msg: Image):
        self._process_image(msg, "cam1", CIN_URI_CAM1, cam_name="CCTV01")

    def cb_image_cam2(self, msg: Image):
        self._process_image(msg, "cam2", CIN_URI_CAM2, cam_name="CCTV02")

    # -------- 공통 처리 함수 -------- #

    def _process_image(self, msg: Image, cam_key: str, cin_uri: str, cam_name: str):
        # 전송 간격 제한 (카메라별로 독립)
        now_msg = self.get_clock().now()
        now = now_msg.seconds_nanoseconds()[0] + now_msg.seconds_nanoseconds()[1] * 1e-9

        if self.min_interval and (now - self.last_pub_ts[cam_key]) < self.min_interval:
            return
        self.last_pub_ts[cam_key] = now

        # ROS Image -> OpenCV BGR
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"[{cam_name}] cv_bridge 변환 실패: {e}")
            return

        # JPEG 인코딩
        ok, enc = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().error(f"[{cam_name}] JPEG 인코딩 실패")
            return

        b64 = base64.b64encode(enc.tobytes()).decode('ascii')

        # oneM2M cin con에 넣을 JSON
        payload = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "encoding": "jpeg",
            "data": b64,
            "camera": cam_name,
        }

        try:
            publish_cin_only(cin_uri, AE_ID, payload)
            self.get_logger().info(
                f"[{cam_name}] cin -> {cin_uri} "
                f"(base64 length={len(b64)})"
            )
        except Exception as e:
            self.get_logger().error(f"[{cam_name}] Mobius cin publish 실패: {e}")


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
