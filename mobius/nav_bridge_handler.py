import math
import queue
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose


def _yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

def _quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)

def _extract_con(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Support both oneM2M notify wrapper and raw con dict."""
    sgn = payload.get('m2m:sgn') or payload.get('sgn')
    if sgn:
        nev = sgn.get('nev', {})
        rep = nev.get('rep', {})
        cin = rep.get('m2m:cin') or rep.get('cin')
        if isinstance(cin, dict) and 'con' in cin:
            return cin['con']
    if 'con' in payload and isinstance(payload['con'], dict):
        return payload['con']
    return payload


class _NavBridgeNode(Node):
    def __init__(self):
        super().__init__('mobius_nav_bridge')
        self._init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._q: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._timer = self.create_timer(0.05, self._process_queue)

    def enqueue(self, con: Dict[str, Any]) -> None:
        self._q.put(con)

    def _process_queue(self) -> None:
        try:
            while True:
                con = self._q.get_nowait()
                self._handle_con(con)
        except queue.Empty:
            return

    def _handle_con(self, con: Dict[str, Any]) -> None:
        try:
            ctype = con.get('type')
            if ctype == 'init_pose':
                pose = con.get('pose', con)
                x = float(pose.get('x', 0.0))
                y = float(pose.get('y', 0.0))
                if 'z' in pose and 'w' in pose:
                    yaw = _quat_to_yaw(float(pose['z']), float(pose['w']))
                else:
                    yaw = float(pose.get('yaw', 0.0))
                self._publish_initial_pose(x, y, yaw)
                self.get_logger().info(f'Init pose x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
            elif ctype == 'navigate_to_pose':
                goal = con.get('goal', con)
                x = float(goal.get('x', 0.0))
                y = float(goal.get('y', 0.0))
                if 'z' in goal and 'w' in goal:
                    yaw = _quat_to_yaw(float(goal['z']), float(goal['w']))
                else:
                    yaw = float(goal.get('yaw', 0.0))
                self._send_nav_goal(x, y, yaw)
            else:
                self.get_logger().warn(f'Unknown command type: {ctype}')
        except Exception as e:
            self.get_logger().error(f'Handle error: {e}')

    def _publish_initial_pose(
        self,
        x: float,
        y: float,
        yaw_rad: float,
        cov_xy_var: float = 0.25,
        cov_yaw_var: float = 0.06853891909122467
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=_yaw_to_quat(yaw_rad)
        )
        cov = [0.0] * 36
        cov[0] = cov_xy_var
        cov[7] = cov_xy_var
        cov[35] = cov_yaw_var
        msg.pose.covariance = cov

        for _ in range(3):
            self._init_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.03)

    def _send_nav_goal(self, x: float, y: float, yaw_rad: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=_yaw_to_quat(yaw_rad)
        )

        self.get_logger().info(f'Send goal x={x:.3f} y={y:.3f} yaw={yaw_rad:.3f}')
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        try:
            result = result_future.result().result
            self.get_logger().info(f'Goal result: {result.result}')
        except Exception as e:
            self.get_logger().error(f'Get result error: {e}')

    def _on_feedback(self, fb_msg):
        try:
            self.get_logger().info(f'Feedback: distance_remaining={fb_msg.feedback.distance_remaining:.2f}')
        except Exception:
            pass


# ------- Public API for external caller (no change to original subscriber) -------

_bridge_started = False
_executor: Optional[SingleThreadedExecutor] = None
_node: Optional[_NavBridgeNode] = None
_thread: Optional[threading.Thread] = None

def init_bridge():
    """Safe to call multiple times."""
    global _bridge_started, _executor, _node, _thread
    if _bridge_started:
        return
    rclpy.init(args=None)
    _node = _NavBridgeNode()
    _executor = SingleThreadedExecutor()
    _executor.add_node(_node)

    def _spin():
        try:
            _executor.spin()
        finally:
            _executor.shutdown()
            _node.destroy_node()
            rclpy.shutdown()

    _thread = threading.Thread(target=_spin, daemon=True)
    _thread.start()
    _bridge_started = True

def process_mobius_payload(payload: Dict[str, Any]) -> None:
    """Entry point to be called by the existing mqtt_subscribe.py stack."""
    if not _bridge_started:
        init_bridge()
    con = _extract_con(payload)
    if isinstance(con, dict) and _node is not None:
        _node.enqueue(con)
        