import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from utils_parse import parse_sensor_line
from mqtt_publish import publish_cin_only

AE_ID = 'CAdmin'
ROS_NS = '/s1'
MOBIUS_BASE = '/Mobius/Meta-Sejong/Chungmu-hall'
SENSORS = ['Sensor1', 'Sensor2', 'Sensor3']
TOPIC_SUFFIX = 'values'

class Ros2ToMobiusBridge(Node):
    def __init__(self):
        super().__init__('ros2_to_mobius_bridge')

        qos = QoSProfile(depth=10)
        self._subs = []

        for sensor in SENSORS:
            idx = sensor[-1]  # '1','2','3'
            ros_topic = f'{ROS_NS}/sensor{idx}/{TOPIC_SUFFIX}'
            mobius_uri = f'{MOBIUS_BASE}/{sensor}/data'

            # 센서마다 별도의 콜백 생성(모비우스 URI 캡처)
            cb = self._make_callback(sensor, mobius_uri)
            sub = self.create_subscription(String, ros_topic, cb, qos)
            self._subs.append(sub)

            self.get_logger().info(f'Subscribed: {ros_topic} → cin → {mobius_uri}')

    def _make_callback(self, sensor_name: str, mobius_uri: str):
        def _cb(msg: String):
            try:
                payload_obj = parse_sensor_line(msg.data)   # 문자열 → dict
                publish_cin_only(mobius_uri, AE_ID, payload_obj)  # cin만 생성
                self.get_logger().info(
                    f'[{sensor_name}] cin published: keys={list(payload_obj.keys())[:6]}..., raw_len={len(msg.data)}'
                )
            except Exception as e:
                self.get_logger().error(f'[{sensor_name}] MQTT publish failed: {e}')
        return _cb

def main(args=None):
    rclpy.init(args=args)
    node = Ros2ToMobiusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()