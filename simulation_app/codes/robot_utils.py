# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import os
import traceback
from omni.isaac.core.utils import prims
from omni.kit.commands import execute
from pxr import Gf, Usd, Sdf

import usdrt
import usdrt.Sdf
import omni.graph.core as og
import omni.usd

CONST_ROBOT_STAGE_PATH = "/world/metacombot"


def deploy_robot(robot_usd_path, robot_start_point):
    # 로봇 Prim 생성
    prims.create_prim(
        CONST_ROBOT_STAGE_PATH,
        "Xform",
        position=robot_start_point,
        usd_path=robot_usd_path,
    )

    # Stage에서 Lidar Prim 가져오기
    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/Velodyne_VLS_128"
    lidar_prim = stage.GetPrimAtPath(lidar_prim_path)

    if lidar_prim.IsValid():
        # ✅ 90도 Z축 회전
        orient_attr = lidar_prim.GetAttribute("xformOp:orient")
        if not orient_attr.IsValid():
            # Attribute 없으면 생성
            orient_attr = lidar_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf, False)

        # 90도 회전
        rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)
        quat = rotation.GetQuat()  # 이 자체가 GfQuatd 타입임

        # Attribute에 바로 적용 (GfQuatd 그대로 사용!)
        orient_attr.Set(quat)

        # 기존 설정
        lidar_helper_path = lidar_prim_path + "/Velodyne/ros2_rtx_lidar_helper"
        lidar_helper_prim = stage.GetPrimAtPath(lidar_helper_path)

        if lidar_helper_prim.IsValid():
            # frameSkipCount 수정
            frame_skip_attr = lidar_helper_prim.GetAttribute("inputs:frameSkipCount")
            if frame_skip_attr.IsValid():
                frame_skip_attr.Set(1)

            # frameId 수정
            frame_id_attr = lidar_helper_prim.GetAttribute("inputs:frameId")
            if frame_id_attr.IsValid():
                frame_id_attr.Set("base_link")
        else:
            pass
    else:
        pass


def set_recycling_checker():
    """재활용품 분류 체크를 위한 OmniGraph 설정"""
    GRAPH_PATH = "/recycling_checker"
    
    try:
        # 1. 그래프 생성과 노드들 생성
        (_, (_, script_node,_,_,_,_,_,_,_,_,_), _, _) = og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("recycling_trigger_node", "omni.physx.graph.OnTriggerCollider"),
                    ("recycling_counter_script_node", "omni.graph.scriptnode.ScriptNode"),
                    ("make_recycled_trash_prim_inactive", "omni.graph.action.SetPrimActive"),
                    ("get_recycled_trash_prim_path", "omni.graph.nodes.ToString"),
                    ("read_total_trash_counter", "omni.graph.nodes.ReadPrimAttribute"),
                    ("read_recycling_counter", "omni.graph.nodes.ReadPrimAttribute"),
                    ("write_recycling_counter", "omni.graph.nodes.WritePrimAttribute"),
                    ("increment_recycling_counter", "omni.graph.nodes.Increment"),
                    ("read_stage2_score", "omni.graph.nodes.ReadPrimAttribute"),
                    ("write_stage2_score", "omni.graph.nodes.WritePrimAttribute"),
                    ("increment_stage2_score", "omni.graph.nodes.Increment"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("read_total_trash_counter.inputs:prim", [usdrt.Sdf.Path("/Scoreboard/recycling")]),
                    ("read_total_trash_counter.inputs:name", "total"),
                    
                    ("read_recycling_counter.inputs:prim", [usdrt.Sdf.Path("/Scoreboard/recycling")]),
                    ("read_recycling_counter.inputs:name", "recycled"),
                    ("write_recycling_counter.inputs:prim", [usdrt.Sdf.Path("/Scoreboard/recycling")]),
                    ("write_recycling_counter.inputs:name", "recycled"),

                    ("read_stage2_score.inputs:prim", [usdrt.Sdf.Path("/Scoreboard/score")]),
                    ("read_stage2_score.inputs:name", "stage2"),
                    ("write_stage2_score.inputs:prim", [usdrt.Sdf.Path("/Scoreboard/score")]),
                    ("write_stage2_score.inputs:name", "stage2"),

                    ("make_recycled_trash_prim_inactive.inputs:active", False),
                ]
            }
        )

        # 2.1 스크립트 코드 정의
        RECYCLING_COUNTER_SCRIPT_CODE = '''

RECYCLABLE_TYPES = {
    "tissue": {"type": "paper"},
    "juice": {"type": "plastic"},
    "disposable_cup": {"type": "plastic"},
    "wood_block": {"type": "none"},
    "mug": {"type": "none"},
    "cracker_box": {"type": "paper"},
    "cola_can": {"type": "aluminum"},
    "master_chef_can": {"type": "aluminum"},
}

RECYCLED_TRASH_PRIMS = []

def compute(db):
    """매 프레임마다 호출되는 메인 로직"""
    try:
        trigger_prim = db.inputs.triggerPrimPath
        entering_prim = db.inputs.enteringPrim
        total_trash_count = db.inputs.totalTrashCount
        
        if not trigger_prim or not entering_prim:
            db.outputs.recyclingScore = 0.0
            db.outputs.recyclingCount = 0

            return True
            
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(entering_prim)
        if not prim.IsValid():
            print(f"Invalid prime path: {entering_prim}")
            db.outputs.recyclingScore = 0.0
            db.outputs.recyclingCount = 0

            return True

        if entering_prim in RECYCLED_TRASH_PRIMS:
            db.outputs.recyclingScore = 0.0
            db.outputs.recyclingCount = 0
            
            return True

        RECYCLED_TRASH_PRIMS.append(entering_prim)

        # trigger_prim에서 마지막 항목 추출
        trigger_parts = trigger_prim.split("/")
        trigger_last = trigger_parts[-1] if trigger_parts else ""
        
        # "trigger_" 접두사 제거하여 trash_category 저장
        trash_category = ""
        if trigger_last.startswith("trigger_"):
            trash_category = trigger_last[len("trigger_"):]

            
        # entering_prim에서 마지막 항목 추출
        entering_parts = entering_prim.split("/")
        entering_last = entering_parts[-1] if entering_parts else ""
        
        # RECYCLABLE_TYPES의 key와 매칭되는 항목 찾기
        recycled_object_type = None
        for key in RECYCLABLE_TYPES:
            if key in entering_last:
                recycled_object_type = RECYCLABLE_TYPES[key]
                break

        # 수집된 object가 재활용 가능한 유형이 아닌경우 점수 계산 안함
        if recycled_object_type and recycled_object_type["type"] == "none":
            db.outputs.recyclingScore = 0.0
            db.outputs.recyclingCount = 0

            return True
        
        # 추가 점수 계산
        additional_score = 0.0
        if recycled_object_type and trash_category == recycled_object_type["type"]:
            additional_score = 5.0

        db.outputs.recyclingScore = (5.0 + additional_score) / (total_trash_count * 10.0) * 100
        db.outputs.recyclingCount = 1

        print(f"recyclingScore: {db.outputs.recyclingScore}, {trigger_prim} {entering_prim}")
        return True
                
    except Exception as e:
        import traceback
        traceback.print_exc()
        return False
'''

        # 2.2 스크립트 노드 속성 설정
        script_attr = script_node.get_attribute(f"inputs:script")
        if script_attr:
            script_attr.set(RECYCLING_COUNTER_SCRIPT_CODE)

        og.Controller.create_attribute(script_node, "enteringPrim", "token", og.AttributePortType.INPUT)
        og.Controller.create_attribute(script_node, "triggerPrimPath", "token", og.AttributePortType.INPUT)
        og.Controller.create_attribute(script_node, "totalTrashCount", "int", og.AttributePortType.INPUT)
        og.Controller.create_attribute(script_node, "recyclingCount", "int", og.AttributePortType.OUTPUT)
        og.Controller.create_attribute(script_node, "recyclingScore", "double", og.AttributePortType.OUTPUT)

        # 3. 노드 간 연결 설정
        # activate node : trigger node -> Set prime active -> script node
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_trigger_node.outputs:enterExecOut",
            f"{GRAPH_PATH}/make_recycled_trash_prim_inactive.inputs:execIn"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/make_recycled_trash_prim_inactive.outputs:execOut",
            f"{GRAPH_PATH}/recycling_counter_script_node.inputs:execIn"
        )

        
        ## trigger node -> script node
        # og.Controller.connect(
        #     f"{GRAPH_PATH}/recycling_trigger_node.outputs:enterExecOut",
        #     f"{GRAPH_PATH}/recycling_counter_script_node.inputs:execIn"
        # )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_trigger_node.outputs:otherBody",
            f"{GRAPH_PATH}/recycling_counter_script_node.inputs:enteringPrim"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_trigger_node.outputs:triggerCollider",
            f"{GRAPH_PATH}/recycling_counter_script_node.inputs:triggerPrimPath"
        )

        ## script node 추가 입력 (trash count)
        og.Controller.connect(
            f"{GRAPH_PATH}/read_total_trash_counter.outputs:value",
            f"{GRAPH_PATH}/recycling_counter_script_node.inputs:totalTrashCount"
        )

        ## recycling count update & write 
        og.Controller.connect(
            f"{GRAPH_PATH}/read_recycling_counter.outputs:value",
            f"{GRAPH_PATH}/increment_recycling_counter.inputs:value"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_counter_script_node.outputs:recyclingCount",
            f"{GRAPH_PATH}/increment_recycling_counter.inputs:increment"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/increment_recycling_counter.outputs:result",
            f"{GRAPH_PATH}/write_recycling_counter.inputs:value"
        )

        ## score update & write 
        og.Controller.connect(
            f"{GRAPH_PATH}/read_stage2_score.outputs:value",
            f"{GRAPH_PATH}/increment_stage2_score.inputs:value"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_counter_script_node.outputs:recyclingScore",
            f"{GRAPH_PATH}/increment_stage2_score.inputs:increment"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/increment_stage2_score.outputs:result",
            f"{GRAPH_PATH}/write_stage2_score.inputs:value"
        )

        ## recycling_counter_script_node execOut -> write prime attribute node
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_counter_script_node.outputs:execOut",
            f"{GRAPH_PATH}/write_stage2_score.inputs:execIn"
        )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_counter_script_node.outputs:execOut",
            f"{GRAPH_PATH}/write_recycling_counter.inputs:execIn"
        )

        ## Make Recycled Trash Prim Inactive
        # og.Controller.connect(
        #     f"{GRAPH_PATH}/recycling_counter_script_node.outputs:execOut",
        #     f"{GRAPH_PATH}/make_recycled_trash_prim_inactive.inputs:execIn"
        # )
        og.Controller.connect(
            f"{GRAPH_PATH}/recycling_trigger_node.outputs:otherBody",
            f"{GRAPH_PATH}/get_recycled_trash_prim_path.inputs:value"
        )   
        og.Controller.connect(
            f"{GRAPH_PATH}/get_recycled_trash_prim_path.outputs:converted",
            f"{GRAPH_PATH}/make_recycled_trash_prim_inactive.inputs:prim"
        )
        
        

        
        # 4. 트리거 관계 설정
        stage_variant = omni.usd.get_context().get_stage()
        prim = stage_variant.GetPrimAtPath(f"{GRAPH_PATH}/recycling_trigger_node")
        execute(
            "SetRelationshipTargets",
            relationship=prim.GetRelationship("inputs:triggersRelationships"),
            targets=[
                Sdf.Path(f"/world/metacombot/trigger/trigger_aluminum"),
                Sdf.Path(f"/world/metacombot/trigger/trigger_plastic"),
                Sdf.Path(f"/world/metacombot/trigger/trigger_paper"),
            ],
        )

        return True

    except Exception as e:
        traceback.print_exc()
        return False


# 로봇팔의 material 설정
def set_robot_arm_material():
    execute(
        "BindMaterial",
        material_path="/world/ObjectFriction",
        prim_path=[
            f"{CONST_ROBOT_STAGE_PATH}/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/left_gripper/D_A03_ASM_DOIGTS_PARALLELES_1ROBOTIQ_HAND_E_DEFEATURE_02",
            f"{CONST_ROBOT_STAGE_PATH}/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/right_gripper/D_A03_ASM_DOIGTS_PARALLELES_1ROBOTIQ_HAND_E_DEFEATURE_02",
        ],
        strength=["weakerThanDescendants", "weakerThanDescendants"],
        material_purpose="",
    )
    try:
        og.Controller.edit(
            {"graph_path": "/pick_and_place", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("gripper_open_value", "omni.graph.nodes.ConstantDouble"),
                    ("gripper_close_value", "omni.graph.nodes.ConstantDouble"),
                    ("gripper_select_if", "omni.graph.nodes.SelectIf"),
                    (
                        "gripper_write_prim_attribute",
                        "omni.graph.nodes.WritePrimAttribute",
                    ),
                    ("gripper_attribute_name", "omni.graph.nodes.ConstantToken"),
                    ("gripper_path", "omni.graph.nodes.ConstantToken"),
                    (
                        "pickandplace_recv_custom_event",
                        "omni.graph.action.OnCustomEvent",
                    ),
                    ("manipulator_path", "omni.graph.nodes.ConstantToken"),
                    (
                        "pick_and_place_node",
                        "aisl.omnigraph.extension.PickandPlaceNode",
                    ),
                    ("pickandplace_read_attribute", "omni.graph.core.ReadVariable"),
                    ("pickandplace_write_attribute", "omni.graph.core.WriteVariable"),
                    ("boolean_or", "omni.graph.nodes.BooleanOr"),
                    ("pickandplace_command_not", "omni.graph.nodes.ConstantToken"),
                    ("pickandplace_command", "omni.graph.nodes.ConstantToken"),
                    ("pickandplace_select_if", "omni.graph.nodes.SelectIf"),
                    (
                        "pickandplace_send_custom_event",
                        "omni.graph.action.SendCustomEvent",
                    ),
                    ("read_target_ori", "omni.graph.nodes.ReadPrimAttribute"),
                    ("target_ori_attribute_name", "omni.graph.nodes.ConstantToken"),
                    (
                        "pick_and_place_cmd_receiver",
                        "omni.graph.scriptnode.ScriptNode",
                    ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "pick_and_place_node.outputs:gripper_grasp_command",
                        "gripper_select_if.inputs:condition",
                    ),
                    (
                        "gripper_close_value.inputs:value",
                        "gripper_select_if.inputs:ifFalse",
                    ),
                    (
                        "gripper_open_value.inputs:value",
                        "gripper_select_if.inputs:ifTrue",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "gripper_write_prim_attribute.inputs:execIn",
                    ),
                    (
                        "gripper_attribute_name.inputs:value",
                        "gripper_write_prim_attribute.inputs:name",
                    ),
                    (
                        "gripper_path.inputs:value",
                        "gripper_write_prim_attribute.inputs:primPath",
                    ),
                    (
                        "gripper_select_if.outputs:result",
                        "gripper_write_prim_attribute.inputs:value",
                    ),
                    (
                        "pickandplace_recv_custom_event.outputs:execOut",
                        "pick_and_place_node.inputs:execution",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:grasp_point_ori",
                        "pick_and_place_node.inputs:grasping_point_ori",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:grasp_point_pos",
                        "pick_and_place_node.inputs:grasping_point_pos",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:placement_point_ori",
                        "pick_and_place_node.inputs:placement_point_ori",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:placement_point_pos",
                        "pick_and_place_node.inputs:placement_point_pos",
                    ),
                    (
                        "manipulator_path.inputs:value",
                        "pick_and_place_node.inputs:robot_prim_path",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "pickandplace_write_attribute.inputs:execIn",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:ppcmd",
                        "pickandplace_write_attribute.inputs:value",
                    ),
                    (
                        "pick_and_place_node.outputs:pick_and_place_command",
                        "boolean_or.inputs:a",
                    ),
                    (
                        "pickandplace_read_attribute.outputs:value",
                        "boolean_or.inputs:b",
                    ),
                    (
                        "boolean_or.outputs:result",
                        "pickandplace_select_if.inputs:condition",
                    ),
                    (
                        "pickandplace_command_not.inputs:value",
                        "pickandplace_select_if.inputs:ifFalse",
                    ),
                    (
                        "pickandplace_command.inputs:value",
                        "pickandplace_select_if.inputs:ifTrue",
                    ),
                    (
                        "pickandplace_select_if.outputs:result",
                        "pickandplace_send_custom_event.inputs:eventName",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "pickandplace_send_custom_event.inputs:execIn",
                    ),
                    (
                        "target_ori_attribute_name.inputs:value",
                        "read_target_ori.inputs:name",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "pick_and_place_cmd_receiver.inputs:execIn",
                    ),
                ],
                og.Controller.Keys.CREATE_VARIABLES: [
                    ("pickandplace", "bool"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    (
                        "pick_and_place_cmd_receiver.outputs:grasp_point_ori",
                        "double[4]",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:grasp_point_pos",
                        "double[3]",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:placement_point_ori",
                        "double[4]",
                    ),
                    (
                        "pick_and_place_cmd_receiver.outputs:placement_point_pos",
                        "double[3]",
                    ),
                    ("pick_and_place_cmd_receiver.outputs:ppcmd", "bool"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("pick_and_place_cmd_receiver.inputs:usePath", True),
                    (
                        "pick_and_place_cmd_receiver.inputs:scriptPath",
                        os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            "ppcmd_receiver.py",
                        ),
                    ),
                    ("gripper_open_value.inputs:value", 0.035),
                    ("gripper_close_value.inputs:value", -0.1),
                    ("gripper_write_prim_attribute.inputs:usePath", True),
                    (
                        "gripper_attribute_name.inputs:value",
                        "drive:linear:physics:targetVelocity",
                    ),
                    (
                        "gripper_path.inputs:value",
                        f"{CONST_ROBOT_STAGE_PATH}/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/Slider_1",
                    ),
                    ("pickandplace_recv_custom_event.inputs:eventName", "pickandplace"),
                    (
                        "manipulator_path.inputs:value",
                        f"{CONST_ROBOT_STAGE_PATH}/metacom_robot/kinova_robot_manipulator",
                    ),
                    ("pickandplace_read_attribute.inputs:variableName", "pickandplace"),
                    (
                        "pickandplace_write_attribute.inputs:variableName",
                        "pickandplace",
                    ),
                    ("pickandplace_command_not.inputs:value", "pickandplace_not"),
                    ("pickandplace_command.inputs:value", "pickandplace"),
                    ("read_target_ori.inputs:usePath", True),
                    ("target_ori_attribute_name.inputs:value", "xformOp:orient"),
                    ("pickandplace_read_attribute.inputs:graph", "/pick_and_place"),
                    ("pickandplace_read_attribute.inputs:variableName", "pickandplace"),
                    ("pickandplace_write_attribute.inputs:graph", "/pick_and_place"),
                    (
                        "pickandplace_write_attribute.inputs:variableName",
                        "pickandplace",
                    ),
                ],
            },
        )
    except Exception as e:
        traceback.print_exc()

    try:
        og.Controller.edit(
            {"graph_path": "/camera", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "ros2_camera_helper_center", 
                        "omni.isaac.ros2_bridge.ROS2CameraHelper"
                    ),
                    (
                        "ros2_camera_helper_center_depth",
                        "omni.isaac.ros2_bridge.ROS2CameraHelper",
                    ),
                    (
                        "isaac_set_camera",
                        "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
                    ),
                    (
                        "isaac_create_render_product",
                        "omni.isaac.core_nodes.IsaacCreateRenderProduct",
                    ),
                    (
                        "isaac_run_one_simulation_frame",
                        "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame",
                    ),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    (
                        "ros2_camera_info_helper_center",
                        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
                    ),
                    ("constant_string", "omni.graph.nodes.ConstantString"),
                    (
                        "isaac_set_left_camera",
                        "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
                    ),
                    (
                        "isaac_set_right_camera",
                        "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
                    ),
                    (
                        "isaac_create_render_product_4_left",
                        "omni.isaac.core_nodes.IsaacCreateRenderProduct",
                    ),
                    (
                        "isaac_create_render_product_4_right",
                        "omni.isaac.core_nodes.IsaacCreateRenderProduct",
                    ),
                    (
                        "ros2_camera_helper_left",
                        "omni.isaac.ros2_bridge.ROS2CameraHelper",
                    ),
                    (
                        "ros2_camera_helper_right",
                        "omni.isaac.ros2_bridge.ROS2CameraHelper",
                    ),
                    (
                        "ros2_camera_info_helper_left",
                        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
                    ),
                    (
                        "ros2_camera_info_helper_right",
                        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
                    ),
                    # (
                    #     "ros2_camera_helper_02",
                    #     "omni.isaac.ros2_bridge.ROS2CameraHelper",
                    # ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "ros2_context.outputs:context",
                        "ros2_camera_helper_center.inputs:context",
                    ),
                    (
                        "isaac_set_camera.outputs:execOut",
                        "ros2_camera_helper_center.inputs:execIn",
                    ),
                    (
                        "constant_string.inputs:value",
                        "ros2_camera_helper_center.inputs:nodeNamespace",
                    ),
                    (
                        "isaac_create_render_product.outputs:renderProductPath",
                        "ros2_camera_helper_center.inputs:renderProductPath",
                    ),
                    (
                        "ros2_context.outputs:context",
                        "ros2_camera_helper_center_depth.inputs:context",
                    ),
                    (
                        "isaac_set_camera.outputs:execOut",
                        "ros2_camera_helper_center_depth.inputs:execIn",
                    ),
                    (
                        "constant_string.inputs:value",
                        "ros2_camera_helper_center_depth.inputs:nodeNamespace",
                    ),
                    (
                        "isaac_create_render_product.outputs:renderProductPath",
                        "ros2_camera_helper_center_depth.inputs:renderProductPath",
                    ),
                    (
                        "isaac_create_render_product.outputs:execOut",
                        "isaac_set_camera.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product.outputs:renderProductPath",
                        "isaac_set_camera.inputs:renderProductPath",
                    ),
                    (
                        "isaac_run_one_simulation_frame.outputs:step",
                        "isaac_create_render_product.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "isaac_run_one_simulation_frame.inputs:execIn",
                    ),
                    (
                        "ros2_context.outputs:context",
                        "ros2_camera_info_helper_center.inputs:context",
                    ),
                    (
                        "isaac_set_camera.outputs:execOut",
                        "ros2_camera_info_helper_center.inputs:execIn",
                    ),
                    (
                        "constant_string.inputs:value",
                        "ros2_camera_info_helper_center.inputs:nodeNamespace",
                    ),
                    (
                        "isaac_create_render_product.outputs:renderProductPath",
                        "ros2_camera_info_helper_center.inputs:renderProductPath",
                    ),
                    # (
                    #     "ros2_context.outputs:context",
                    #     "ros2_camera_helper_02.inputs:context",
                    # ),
                    # (
                    #     "isaac_set_camera.outputs:execOut",
                    #     "ros2_camera_helper_02.inputs:execIn",
                    # ),
                    # (
                    #     "constant_string.inputs:value",
                    #     "ros2_camera_helper_02.inputs:nodeNamespace",
                    # ),
                    # (
                    #     "isaac_create_render_product.outputs:renderProductPath",
                    #     "ros2_camera_helper_02.inputs:renderProductPath",
                    # ),
                    (
                        "isaac_run_one_simulation_frame.outputs:step",
                        "isaac_create_render_product_4_left.inputs:execIn",
                    ),
                    (
                        "isaac_run_one_simulation_frame.outputs:step",
                        "isaac_create_render_product_4_right.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product_4_left.outputs:execOut",
                        "isaac_set_left_camera.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product_4_right.outputs:execOut",
                        "isaac_set_right_camera.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product_4_left.outputs:renderProductPath",
                        "ros2_camera_helper_left.inputs:renderProductPath",
                    ),
                    (
                        "isaac_create_render_product_4_right.outputs:renderProductPath",
                        "ros2_camera_helper_right.inputs:renderProductPath",
                    ),
                    (
                        "isaac_set_left_camera.outputs:execOut",
                        "ros2_camera_helper_left.inputs:execIn",
                    ),
                    (
                        "isaac_set_left_camera.outputs:execOut",
                        "ros2_camera_info_helper_left.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product_4_left.outputs:renderProductPath",
                        "ros2_camera_info_helper_left.inputs:renderProductPath",
                    ),
                    (
                        "isaac_set_right_camera.outputs:execOut",
                        "ros2_camera_helper_right.inputs:execIn",
                    ),
                    (
                        "isaac_set_right_camera.outputs:execOut",
                        "ros2_camera_info_helper_right.inputs:execIn",
                    ),
                    (
                        "isaac_create_render_product_4_right.outputs:renderProductPath",
                        "ros2_camera_info_helper_right.inputs:renderProductPath",
                    ),
                    (
                        "ros2_context.outputs:context",
                        "ros2_camera_helper_left.inputs:context",
                    ),
                    (
                        "ros2_context.outputs:context",
                        "ros2_camera_helper_right.inputs:context",
                    ),
                    (
                        "constant_string.inputs:value",
                        "ros2_camera_helper_left.inputs:nodeNamespace",
                    ),
                    (
                        "constant_string.inputs:value",
                        "ros2_camera_helper_right.inputs:nodeNamespace",
                    ),
                    (
                        "isaac_create_render_product_4_left.outputs:renderProductPath",
                        "isaac_set_left_camera.inputs:renderProductPath",
                    ),
                    (
                        "isaac_create_render_product_4_right.outputs:renderProductPath",
                        "isaac_set_right_camera.inputs:renderProductPath",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ros2_camera_helper_center.inputs:frameId", "Camera"),
                    ("ros2_camera_helper_center.inputs:topicName", "robot/center_camera_image"),
                    ("ros2_camera_helper_center_depth.inputs:frameId", "Camera"),
                    ("ros2_camera_helper_center_depth.inputs:topicName", "robot/center_camera_depth"),
                    ("ros2_camera_helper_center_depth.inputs:type", "depth"),
                    ("ros2_camera_info_helper_center.inputs:frameId", "Camera"),
                    ("ros2_camera_info_helper_center.inputs:topicName", "robot/center_camera_info"),
                    ("constant_string.inputs:value", "metasejong2025"),
                    ("ros2_camera_helper_right.inputs:frameId", "CameraRight"),
                    ("ros2_camera_helper_right.inputs:topicName", "robot/right_camera_image"),
                    ("ros2_camera_helper_left.inputs:frameId", "CameraLeft"),
                    ("ros2_camera_helper_left.inputs:topicName", "robot/left_camera_image"),
                    ("ros2_camera_info_helper_left.inputs:frameId", "CameraLeft"),
                    ("ros2_camera_info_helper_left.inputs:nodeNamespace", "metasejong2025"),
                    ("ros2_camera_info_helper_left.inputs:topicName", "robot/left_camera_info"),
                    ("ros2_camera_info_helper_right.inputs:frameId", "CameraRight"),
                    ("ros2_camera_info_helper_right.inputs:nodeNamespace", "metasejong2025"),
                    ("ros2_camera_info_helper_right.inputs:topicName", "robot/right_camera_info"),
                    (
                        "isaac_create_render_product.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/Camera"
                            )
                        ],
                    ),
                    (
                        "isaac_set_camera.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/Camera"
                            )
                        ],
                    ),
                    (
                        "isaac_create_render_product_4_left.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraLeft"
                            )
                        ],
                    ),
                    (
                        "isaac_set_left_camera.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraLeft"
                            )
                        ],
                    ),
                    (
                        "isaac_create_render_product_4_right.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraRight"
                            )
                        ],
                    ),
                    (
                        "isaac_set_right_camera.inputs:cameraPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraRight"
                            )
                        ],
                    ),
                ],
            },
        )
 
    except Exception as e:
        traceback.print_exc()
 
    try:
        og.Controller.edit(
            {"graph_path": "/camera_tf", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "ros2_publish_transform_tree",
                        "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
                    ),
                    (
                        "isaac_read_simulation_time",
                        "omni.isaac.core_nodes.IsaacReadSimulationTime",
                    ),
                    (
                        "camera_tf_world",
                        "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree",
                    ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "on_playback_tick.outputs:tick",
                        "ros2_publish_transform_tree.inputs:execIn",
                    ),
                    (
                        "isaac_read_simulation_time.outputs:simulationTime",
                        "ros2_publish_transform_tree.inputs:timeStamp",
                    ),
                    ("on_playback_tick.outputs:tick", "camera_tf_world.inputs:execIn"),
                    (
                        "isaac_read_simulation_time.outputs:simulationTime",
                        "camera_tf_world.inputs:timeStamp",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "ros2_publish_transform_tree.inputs:parentPrim",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/scout_v2_base/base_link"
                            )
                        ],
                    ),
                    (
                        "ros2_publish_transform_tree.inputs:targetPrims",
                        [
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/Camera"
                            ),
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraLeft"
                            ),
                            usdrt.Sdf.Path(
                                f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/ZED_X/CameraRight"
                            ),
                        ],
                    ),
                    # ("ros2_publish_transform_tree.inputs:nodeNamespace", "metasejong2025"),
                    # ("camera_tf_world.inputs:nodeNamespace", "metasejong2025"),
                    ("camera_tf_world.inputs:topicName", "/tf"),
                    ("camera_tf_world.inputs:parentFrameId", "Camera"),
                    ("camera_tf_world.inputs:childFrameId", "Camera_world"),
                    ("camera_tf_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
            },
        )
    except Exception as e:
        traceback.print_exc()


def set_robot_namespace(namespace:str):
    """
    로봇의 ROS2 네임스페이스와 토픽명을 설정합니다.
    namespace가 빈 문자열("")이면 표준 ROS 토픽명을 사용합니다.
    """
    stage = omni.usd.get_context().get_stage()
    
    # 표준 ROS 토픽 이름 사용
    topic_mappings = {
        f"{CONST_ROBOT_STAGE_PATH}/scout_v2_base/Drive_front_plugin/ros2_subscribe_twist": "/cmd_vel",
        f"{CONST_ROBOT_STAGE_PATH}/scout_v2_base/Drive_rear_plugin/ros2_subscribe_twist": "/cmd_vel",
        f"{CONST_ROBOT_STAGE_PATH}/scout_v2_base/Odom_plugin/ros2_publish_odometry": "/odom",
    }
    
    # Lidar scan 토픽 설정
    lidar_helper_path = f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/Velodyne_VLS_128/Velodyne/ros2_rtx_lidar_helper"
    lidar_helper_prim = stage.GetPrimAtPath(lidar_helper_path)
    if lidar_helper_prim.IsValid():
        topic_attr = lidar_helper_prim.GetAttribute("inputs:topicName")
        if not topic_attr.IsValid():
            topic_attr = lidar_helper_prim.CreateAttribute("inputs:topicName", Sdf.ValueTypeNames.String, False)
        topic_attr.Set("/scan")
        
        # 네임스페이스도 비우기
        namespace_attr = lidar_helper_prim.GetAttribute("inputs:nodeNamespace")
        if namespace_attr.IsValid():
            namespace_attr.Set("")
    
    # cmd_vel과 odom 토픽 설정
    for prim_path, topic_name in topic_mappings.items():
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            # 토픽명 설정
            topic_attr = prim.GetAttribute("inputs:topicName")
            if not topic_attr.IsValid():
                topic_attr = prim.CreateAttribute("inputs:topicName", Sdf.ValueTypeNames.String, False)
            topic_attr.Set(topic_name)
            
            # 네임스페이스 설정 (빈 문자열로)
            namespace_attr = prim.GetAttribute("inputs:nodeNamespace")
            if namespace_attr.IsValid():
                namespace_attr.Set("")
    
    # Clock 토픽도 표준으로
    clock_prim_path = f"{CONST_ROBOT_STAGE_PATH}/scout_v2_base/ROS_clock_plugin/ros2_publish_clock"
    clock_prim = stage.GetPrimAtPath(clock_prim_path)
    if clock_prim.IsValid():
        namespace_attr = clock_prim.GetAttribute("inputs:nodeNamespace")
        if namespace_attr.IsValid():
            namespace_attr.Set("")


    constant_node_prims = [
        f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/Velodyne_VLS_128/Velodyne/constant_string",
    ]
    for prim_path in constant_node_prims:
        prim = stage.GetPrimAtPath(prim_path)
        
        namespace_attr = prim.GetAttribute("inputs:value")
        if namespace_attr.IsValid():
            # Attribute 없으면 생성
           namespace_attr.Set(namespace)


############ deprecated functions START############


# [DEPRECATED] publish_robot()함수가 deprecated 되고,  deploy_robot()함수로 대체됨
def publish_robot(usd_path, robot_start_point):
    # 로봇 Prim 생성
    prims.create_prim(
        "/world/metacombot",
        "Xform",
        position=robot_start_point,
        usd_path=usd_path,
    )

    # Stage에서 Lidar Prim 가져오기
    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = f"{CONST_ROBOT_STAGE_PATH}/texture_reflected_mount_v2/Velodyne_VLS_128"
    lidar_prim = stage.GetPrimAtPath(lidar_prim_path)

    if lidar_prim.IsValid():
        # ✅ 90도 Z축 회전
        orient_attr = lidar_prim.GetAttribute("xformOp:orient")
        if not orient_attr.IsValid():
            # Attribute 없으면 생성
            orient_attr = lidar_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf, False)

        # 90도 회전
        rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)
        quat = rotation.GetQuat()  # 이 자체가 GfQuatd 타입임

        # Attribute에 바로 적용 (GfQuatd 그대로 사용!)
        orient_attr.Set(quat)


        # 기존 설정
        lidar_helper_path = lidar_prim_path + "/Velodyne/ros2_rtx_lidar_helper"
        lidar_helper_prim = stage.GetPrimAtPath(lidar_helper_path)

        if lidar_helper_prim.IsValid():
            # frameSkipCount 수정
            frame_skip_attr = lidar_helper_prim.GetAttribute("inputs:frameSkipCount")
            if frame_skip_attr.IsValid():
                frame_skip_attr.Set(1)

            # frameId 수정
            frame_id_attr = lidar_helper_prim.GetAttribute("inputs:frameId")
            if frame_id_attr.IsValid():
                frame_id_attr.Set("base_link")
        else:
            pass
    else:
        pass

