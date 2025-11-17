# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import numpy as np

# 여기서는 rclpy.init()와 shutdown() 호출하지 않음.
from ros2_shared import init_ros_node

ros_node = init_ros_node("omnigraph_subscriber")  # 공유 노드 사용
last_received_data = None
ppcmd_active = False
ppcmd_timer = 0.0


def string_callback(msg):
    global last_received_data, ppcmd_active, ppcmd_timer
    try:
        floats = list(map(float, msg.data.strip().split()))
        if len(floats) != 14:
            print(f"[Competitor->Platform] Pick-and-place command: Error - Received data validation error: {len(floats)}")
            return
        last_received_data = floats
        ppcmd_active = True
        ppcmd_timer = time.time()
        print(f"[Competitor->Platform] Pick-and-place command: Success")
    except Exception as e:
        print(f"[Competitor->Platform] Pick-and-place command: Error - Unknown error: {e}")
        traceback.print_exc()


def setup(db):
    ros_node.create_subscription(String, "/metasejong2025/robot/ppcmd", string_callback, 10)


def compute(db):
    global last_received_data, ppcmd_active, ppcmd_timer
    if last_received_data:
        db.outputs.grasp_point_ori = np.array(last_received_data[0:4])
        db.outputs.grasp_point_pos = np.array(last_received_data[4:7])
        db.outputs.placement_point_ori = np.array(last_received_data[7:11])
        db.outputs.placement_point_pos = np.array(last_received_data[11:14])
    if ppcmd_active and (time.time() - ppcmd_timer <= 1.0):
        db.outputs.ppcmd = True
    else:
        db.outputs.ppcmd = False
        ppcmd_active = False
    return True


def cleanup(db):
    pass
