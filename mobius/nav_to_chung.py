import json
import threading
import configparser
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion

INI_PATH = '/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini'

config = configparser.ConfigParser()
if not config.read(INI_PATH):
    raise FileNotFoundError(f"config.ini를 찾을 수 없습니다: {INI_PATH}")

IOTPLATFORM_IP = config['API']['IOTPLATFORM_IP']
IOTPLATFORM_MQTT_PORT = int(config['API']['IOTPLATFORM_MQTT_PORT'])
MQTT_TOPIC = "/oneM2M/req/Mobius2/CampusEMS-Robot-Robot01-Control-Nav/json"


# ============ ROS2 Nav2 브릿지 노드 ============

class NavToChungBridge(Node):
    def __init__(self):
        super().__init__("nav_to_chung_bridge")
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("NavToChungBridge node started.")

    def _parse_pose_from_con(self, con: dict) -> PoseStamped:
        pose_dict = con.get("pose", {})

        header = pose_dict.get("header", {})
        pose_inner = pose_dict.get("pose", {})

        pos = pose_inner.get("position", {})
        ori = pose_inner.get("orientation", {})

        msg = PoseStamped()
        msg.header.frame_id = header.get("frame_id", "map")
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(pos.get("x", 0.0))
        msg.pose.position.y = float(pos.get("y", 0.0))
        msg.pose.position.z = float(pos.get("z", 0.0))

        q = Quaternion()
        q.x = float(ori.get("x", 0.0))
        q.y = float(ori.get("y", 0.0))
        q.z = float(ori.get("z", 0.0))
        q.w = float(ori.get("w", 1.0))
        msg.pose.orientation = q

        return msg

    def send_nav_goal_from_con(self, con: dict):
        try:
            pose_stamped = self._parse_pose_from_con(con)
        except Exception as e:
            self.get_logger().error(f"Failed to parse pose from con: {e}")
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Sending Nav goal: "
            f"x={pose_stamped.pose.position.x:.3f}, "
            f"y={pose_stamped.pose.position.y:.3f}, "
            f"frame={pose_stamped.header.frame_id}"
        )

        send_future = self._nav_client.send_goal_async(goal_msg)

        def _goal_response_cb(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Nav goal rejected.")
                return
            self.get_logger().info("Nav goal accepted.")

            result_future = goal_handle.get_result_async()

            def _result_cb(rfut):
                result = rfut.result().result
                self.get_logger().info(f"Nav result received: {result}")
            result_future.add_done_callback(_result_cb)

        send_future.add_done_callback(_goal_response_cb)


_ros_node: NavToChungBridge = None


# ============ MQTT 콜백 ============

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"[MQTT] Connected rc={rc}")
    client.subscribe(MQTT_TOPIC)
    print(f"[MQTT] Subscribed: {MQTT_TOPIC}")


def on_message(client, userdata, msg):
    global _ros_node
    print(f"[MQTT] msg from {msg.topic}: {msg.payload[:200]!r}")

    try:
        payload_str = msg.payload.decode("utf-8")
        payload = json.loads(payload_str)

        # oneM2M Notify 구조에서 cin/con 뽑기
        cin = (
            payload.get("pc", {})
                   .get("m2m:sgn", {})
                   .get("nev", {})
                   .get("rep", {})
                   .get("m2m:cin", {})
        )
        con = cin.get("con")

        print("\n[DEBUG] ==== con ====")
        print(json.dumps(con, ensure_ascii=False, indent=2))
        print("====================================\n")

        if _ros_node is None:
            print("[Bridge] ROS node not ready yet.")
            return

        # 여기서 con 을 기반으로 로봇에게 네비게이션 명령 전송
        _ros_node.send_nav_goal_from_con(con)

    except Exception as e:
        print("[Bridge] on_message error:", e)


# ============ main ============

def main():
    global _ros_node

    # 1) ROS2 초기화 및 노드 시작
    rclpy.init()
    _ros_node = NavToChungBridge()

    # ROS2 스핀을 별도 스레드에서 돌림
    ros_thread = threading.Thread(target=rclpy.spin, args=(_ros_node,), daemon=True)
    ros_thread.start()

    # 2) MQTT 클라이언트 설정
    client = mqtt.Client(client_id="Robot01-NavBridge")
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    print("[Bridge] MQTT loop start")

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        client.disconnect()
        _ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
