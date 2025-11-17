# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import omni.graph.core as og
from pxr import Gf, UsdGeom
import numpy as np
from scipy.spatial.transform import Rotation as R
import omni.replicator.core as rep
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path, set_prim_property
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.syntheticdata._syntheticdata as sd
import omni.usd
from omni.isaac.ros2_bridge import read_camera_info
from scipy.spatial.transform import Rotation as R
import os
from omni.kit.commands import execute

# -------------------------
# 카메라 특성(초점거리, 해상도)과 위치, 회전 설정
# -------------------------
def setup_camera_spec_and_position(configurations):
    camera_configs = configurations.get_mission_camera_configs()
    resolution = configurations.get_fixed_camera_defaults()['default_camera_resolution']

    resources_dir = configurations.get_resources_directory_path()
    # 미션 오브젝트의 USD 파일이 저장된 경로
    mission_object_asset_path = os.path.join(resources_dir, "assets")
    camera_asset_usd_path = os.path.join(mission_object_asset_path, "SM_CCTV_1.usd")

    cam_list = []
    for cam in camera_configs:
        prim_path = "/world/cameras/"+cam['name']
        camera = Camera(
            prim_path=prim_path,
            position=np.array(cam['position']),
            frequency=1,
            resolution=(resolution[0], resolution[1]),
        )

        camera.set_focal_length(cam['focal_length'])

        quaternion = set_camera_absolute_orientation(prim_path, cam['rotation'], degrees=True, order="zyx")
        set_camera_horizontal_aperture(prim_path, cam['horizontal_aperture'])
        cam_list.append(
            {
                'name': cam['name'],
                'camera': camera
            }
        )

        execute(
            "IsaacSimSpawnPrim",
            usd_path=camera_asset_usd_path,
            prim_path=f"{prim_path}/cctv_camera",
            translation=cam['position'],
            rotation=quaternion
        )
        execute(
            "IsaacSimScalePrim",
            prim_path=f"{prim_path}/cctv_camera",
            scale=[2.0, 2.0, 2.0],
        )

    for cam in cam_list:
        cam['camera'].initialize()
    
    return cam_list


def set_distance_to_camera(cameras):
    depth_annotators = {}
    for cam in cameras:
        camera = cam['camera']
        name = cam['name']

        camera.initialize()
        render_product_path = rep.create.render_product(camera.prim_path, camera._resolution)
        camera._render_product_path = render_product_path
        depth_annotator = rep.annotators.get("distance_to_camera")
        camera.add_distance_to_camera_to_frame()
        depth_annotator.attach([render_product_path])
        depth_annotators[camera.prim_path] = depth_annotator

    return depth_annotators


def publish_ros2_camera_data(cam_list):
    approx_freq = 120
    for idx, cam in enumerate(cam_list):
        camera = cam['camera']
        name = cam['name']
        publish_rgb(camera, approx_freq, name)
        publish_camera_info(camera, approx_freq, name)
        publish_camera_tf(camera)

def pixel_to_camera(u, v, depth, fx, fy, cx, cy):
    """
    픽셀 좌표를 카메라 좌표계로 변환
    카메라 좌표계:
    - X: Right (+)
    - Y: Down (+)
    - Z: Forward (+)
    """
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

def camera_to_world_matrix(point_cam, position, quaternion):
    """
    Isaac Sim 카메라 좌표계에서 월드 좌표계로 변환
    World 좌표계:
    - X: Forward (+)
    - Y: Right (+)
    - Z: Up (+)
    
    quaternion: [w, x, y, z] 형식
    """
    # 카메라 좌표계를 월드 좌표계로 변환
    point_world = np.array([point_cam[2], -point_cam[0], -point_cam[1]])
    
    # Isaac Sim 쿼터니언 적용 [w,x,y,z]
    w, x, y, z = quaternion
    rotation = R.from_quat([x, y, z, w])
    
    # 회전 적용 후 위치 더하기
    rotated_point = rotation.apply(point_world)
    world_point = rotated_point + position
    
    return world_point

def set_camera_absolute_orientation(camera_prim_path, euler_angles, degrees=True, order="zyx"):
    camera_prim = get_prim_at_path(camera_prim_path)
    if camera_prim is None:
        return
    quaternion = R.from_euler(order, [euler_angles[2],euler_angles[1],euler_angles[0]], degrees=degrees).as_quat()
    x, y, z, w = quaternion
    gf_quat = Gf.Quatd(w, Gf.Vec3d(x, y, z))
    set_prim_property(camera_prim_path, "xformOp:orient", gf_quat)

    return quaternion

def set_camera_horizontal_aperture(camera_prim_path: str, aperture_mm: float):
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if not camera_prim.IsValid():
        return
    usd_camera = UsdGeom.Camera(camera_prim)
    usd_camera.GetHorizontalApertureAttr().Set(aperture_mm)
    desired_aspect_ratio = 16 / 9  
    new_vertical_aperture = aperture_mm / desired_aspect_ratio
    usd_camera.GetVerticalApertureAttr().Set(new_vertical_aperture)

def get_camera_sensor_size(camera_prim_path):
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if not camera_prim.IsValid():
        raise ValueError(f"Camera path {camera_prim_path} is invalid.")
    usd_camera = UsdGeom.Camera(camera_prim)  
    sensor_width = usd_camera.GetHorizontalApertureAttr().Get()
    sensor_height = usd_camera.GetVerticalApertureAttr().Get()
    return sensor_width, sensor_height

def publish_rgb(camera: Camera, freq, name):
    render_product = camera._render_product_path
    step_size = 1
    topic_name = f"cameras/{name}/image_raw"
    # topic_name = "image_raw"
    queue_size = 100
    node_namespace = "metasejong2025"
    # 마지막부터 2번째까지 가져오기
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_camera_info(camera: Camera, freq, name):
    render_product = camera._render_product_path
    step_size = 1
    topic_name = f"cameras/{name}/camera_info"
    queue_size = 100
    node_namespace = "metasejong2025"
    frame_id = camera.prim_path.split("/")[-1]

    camera_prim_path = camera.prim_path
    sensor_width, sensor_height = get_camera_sensor_size(camera_prim_path)
    focal_length = camera.get_focal_length() * 10

    camera_info = read_camera_info(render_product_path=render_product)

    fx = (focal_length / sensor_width) * camera_info["width"]
    fy = (focal_length / sensor_height) * camera_info["height"]
    cx, cy = camera_info["width"] / 2, camera_info["height"] / 2
    k_matrix = np.array([fx, 0, cx,  0, fy, cy,  0, 0, 1], dtype=np.float32)

    writer = rep.writers.get("ROS2PublishCameraInfo")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=k_matrix.reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_camera_tf(camera: Camera):
    camera_prim = camera.prim_path
    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        camera_frame_id = camera_prim.split("/")[-1]
        ros_camera_graph_path = "/CameraTFActionGraph"

        if not is_prim_path_valid(ros_camera_graph_path):
            og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("RosPublisher.inputs:nodeNamespace", "metasejong2025"),
                    ]
                }
            )

        position, quaternion = camera.get_world_pose()  # quaternion: [w, x, y, z]
        
        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishTF_"+camera_frame_id, "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ("PublishTF_"+camera_frame_id+".inputs:nodeNamespace", "metasejong2025"),
                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),
                    ("PublishTF_"+camera_frame_id+".inputs:parentFrameId", "world"),
                    ("PublishTF_"+camera_frame_id+".inputs:childFrameId", camera_frame_id),
                    ("PublishTF_"+camera_frame_id+".inputs:translation", position),
                    ("PublishTF_"+camera_frame_id+".inputs:rotation", [quaternion[1],quaternion[2],quaternion[3],quaternion[0]]),  # [w, x, y, z] 형식 그대로 사용
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path+"/OnTick.outputs:tick", "PublishTF_"+camera_frame_id+".inputs:execIn"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime", "PublishTF_"+camera_frame_id+".inputs:timeStamp"),
                ],
            },
        )

    except Exception as e:
        raise e