# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

# ros2_shared.py
import rclpy
from rclpy.node import Node
import threading

from common_utils import (
    check_missing_mandatory_key,
    generate_md5_hash,
    validate_md5_hash
)


_ros_initialized = False
_shared_node = None

# 공유 ROS2 Node 초기화. 공유 ROS2 Node는 omni.kit에서 사용하는 ROS2 Node와 동일한 노드인 것 같음 
def init_ros_node(node_name="shared_ros_node"):
    global _ros_initialized, _shared_node
    if not _ros_initialized:
        rclpy.init()

        _shared_node = Node(node_name)
        # 백그라운드 스레드에서 spin 실행 (daemon=True)
        threading.Thread(target=rclpy.spin, args=(_shared_node,), daemon=True).start()

        _ros_initialized = True

    return _shared_node


def stop_ros_node():
    global _ros_initialized, _shared_node

    if _ros_initialized:
        _shared_node.destroy_node()
        rclpy.shutdown()
        _ros_initialized = False
