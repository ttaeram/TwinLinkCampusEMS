# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#


# -------------------------
# SimulationApp 초기화
#   ** [중요] SimulationApp()을 호출한 다음에 omni.xxxx 모듈 사용해야 함. import 하는 순서 중요함.
# -------------------------
import traceback
from isaacsim import SimulationApp

ISSAC_INIT_CONFIG = {
    "renderer": "RayTracedLighting", 
    "headless": False,          # GUI 모드로 실행
    # "width": 1920,             # 화면 너비
    # "height": 1080,            # 화면 높이
    # "window_width": 1920,      # 창 너비
    # "window_height": 1080,     # 창 높이
    # "display_options": 3074,   # FULLSCREEN | WINDOW_HIDE_MENU | WINDOW_HIDE_TOOLBAR
    "physics_dt": 1.0 / 60.0,  # 물리 시뮬레이션 시간 간격
    "physics_substeps": 1,     # 물리 시뮬레이션 서브스텝
    "physics_auto_apply_apis": False  # PhysicsBodyAPI 자동 적용 비활성화
}

simulation_app = SimulationApp(ISSAC_INIT_CONFIG)

# viewport 초기화를 위한 update
simulation_app.update()

# -------------------------
# 시뮬레이션 관련 모듈 임포트 (SimulationApp 이후에 로드)
# -------------------------
import omni.graph.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
from omni.isaac.nucleus import get_assets_root_path
import omni.kit.viewport.utility as vp_util

from omni.isaac.sensor import Camera
from pxr import UsdPhysics, UsdGeom, Sdf
import omni.kit.commands
import omni.usd
import omni.graph.core


# -------------------------
# Flowchart 정의에 따른 경연대회 서버 실행 (@see /docs/MetaSejong-Embodied-AI-competition-platform.drawio)
# -------------------------

#1. 경연 시나리오 데이터 생성 또는 load (경연 대상 지역, 시작 지점, 쓰레기 수/종류/위치, CCTV 수/위치/자세)

#1-1. 설정 파일 읽기 
## 설정, 경진대회 참가자로부터의 요청 처리용 클래스 import
from configurations import Configurations

try:
    configurations = Configurations()
except Exception as e:
    traceback.print_exc()
    raise e


#2. world 초기화, extension 설정, ROS2 초기화 등
#2-1 enable {ROS2 bridge, omnigraph} extension 활성화 
extensions.enable_extension("omni.isaac.ros2_bridge")
# extensions.enable_extension("aisl.omnigraph.extension")  # 로봇팔 미사용으로 제거

simulation_app.update()


# ------------------------- 
# ROS2 관련 모듈 로드 및 초기화 (SimulationApp 인스턴스 이후)
# -------------------------
#2-3 ROS2 초기화
from ros2_shared import init_ros_node, stop_ros_node

try: 
    # 한 번만 초기화. 이미 다른 ScriptNode들도 이 노드를 사용할 수 있음.
    ros_node = init_ros_node("metacom_main_ros_node")

    # 대회 관련 프로세스 제거
    # from competitor_request_handler import CompetitorRequestHandler
    # handler = CompetitorRequestHandler(ros_node)
    # handler.start_ros2()
except Exception as e:
    traceback.print_exc()
    raise e

#2-4 START ########################  WORLD INIT #########################

# -------------------------
# 월드 초기화 및 설정
# -------------------------
from world_utils import (
    setup_world_stage_and_viewport,
    init_world_stage,
)

# 시나리오 정의에서 playground usd path와 viewport 정보 읽어오기
world_usd_path = configurations.get_playground_usd_path()
view_point_camera_view_eye, view_point_camera_view_target = configurations.get_view_point()

# world 초기화 함수 호출
# world stage에 Meta-Sejong USD파일 loading하고, 초기 viewport 설정
setup_world_stage_and_viewport(
    view_point_camera_view_eye,
    view_point_camera_view_target,
    world_usd_path,
)
simulation_app.update()

# world에 state 구성 (collision_api, xformable, dome_light)
init_world_stage()
simulation_app.update()

#2-4 END ########################  WORLD INIT #########################




# -------------------------
# Mission Object(재활용 쓰레기) 초기화 - 비활성화
# -------------------------
#3-1 START ####################### OBJECT INIT (Trash) ########################
# 오브젝트 배치 비활성화
print("[INFO] Object deployment disabled - no trash objects will be placed")
# from object_utils import (
#     set_friction,
#     deploy_mission_objects,
#     set_mission_object_semantics_attributes,
#     set_mission_object_physics,
# )
# 
# # world에 배치될 모든 object들의 마찰 계수 설정
# set_friction()
# simulation_app.update()
# 
# mission_objects_prim_paths, answer_sheet = deploy_mission_objects(
#     stage,
#     configurations
# )
# simulation_app.update()
# 
# # 채점을 위해 정답지를 설정
# # handler.set_answer_sheet(answer_sheet)  # 대회 프로세스 제거로 비활성화
# 
# 
# # 배치된 mission object들의 시멘틱 레이블 설정
# #
# # [DEPRECATED] set_object_attributes_semantics()함수가 deprecated 되고,  set_mission_object_semantics_attributes()함수에서 대체됨
# set_mission_object_semantics_attributes(configurations)
# simulation_app.update()
# 
# # 배치된 mission object들의 물리 속성 설정
# #
# # [DEPRECATED] set_object_physics()함수가 deprecated 되고,  set_mission_object_physics()함수에서 대체됨
# set_mission_object_physics(mission_objects_prim_paths)
# simulation_app.update()

#3-1 END ####################### OBJECT INIT (Trash) ########################

# -------------------------
# 카메라 초기화 - 비활성화
# -------------------------
#3-3 START ####################### CAMERA INIT ########################
# 카메라 설정 비활성화
print("[INFO] Camera setup disabled - no camera sensors will be placed")
# from camera_utils import (
#     setup_camera_spec_and_position,
#     publish_ros2_camera_data
# )
# 
# # 여기에서는 카메라 spec 설정과 위치 설정만하고, 실제 ROS publish 시작은 참가자 메시지 수신 이후에 시작 
# cam_list = setup_camera_spec_and_position(configurations) 
# simulation_app.update()

cam_list = []  # 빈 리스트로 초기화

#3-3 END ####################### CAMERA INIT ########################

# -------------------------
# 로봇 초기화
# -------------------------
#3-3 START ####################### ROBOT INIT ########################
from robot_utils import (
    deploy_robot,
    set_robot_arm_material,
    set_robot_namespace,
    # set_recycling_checker,
)

# 로봇 usd asset 정보와 시작 위치 조회 
robot_usd = configurations.get_robot_usd()
robot_usd_path = configurations.get_robot_usd_path(robot_usd.get("usd_file"))
robot_start_point = configurations.get_robot_start_point()
                         
# 로봇 배치
#
# [DEPRECATED] publish_robot()함수가 deprecated 되고,  deploy_robot()함수로 대체됨
# publish_robot(ROBOT_USD_PATH, robot_start_point)
# deploy_robot(robot_usd_path, robot_start_point)
# set_robot_arm_material()  # 로봇팔 미사용으로 제거
# set_robot_namespace(configurations.get_competitor_interface_namespace())  # 네임스페이스 제거
# set_robot_namespace("")  # 빈 네임스페이스 설정으로 표준 토픽명 사용 - USD에 해당 prim 없음
# set_recycling_checker()
simulation_app.update()
#3-3 END ####################### ROBOT INIT ########################



# -------------------------
# 스코어(점수) 시스템 초기화 - 비활성화
# -------------------------
######################## SCORE INIT #########################
print("[INFO] Scoreboard system disabled")
# from score_utils import (
#     setup_scoreboard, 
#     update_scoreboard,
#     set_scoreboard_recycling,
#     set_scoreboard_score_stage1,
#     set_scoreboard_score_stage2,
#     set_scoreboard_team_info,
# )
# 
# setup_scoreboard(simulation_app)
######################## SCORE INIT #########################




# -------------------------
# 물리 초기화 및 시뮬레이션 실행
# -------------------------

# SimulationContext 초기화 (물리 엔진 시작 전)
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# 물리 엔진 초기화 및 시작
simulation_context.initialize_physics()
simulation_context.play()




from competitor_request_message import MessageType


# recycled, total_recyclable = handler.get_recycled_status()  # 대회 프로세스 제거
recycled, total_recyclable = 0, 0  # 기본값으로 초기화
# set_scoreboard_recycling(recycled, total_recyclable)  # 스코어보드 비활성화

time_elapsed = 0

COMPETITION_STATUS_READY = 0
COMPETITION_STATUS_IN_PROGRESS = 1
COMPETITION_STATUS_FINISHING = 2
COMPETITION_STATUS_FINISHED = 3


comeptition_status = COMPETITION_STATUS_READY
time_constraint = configurations.get_scenario_info().get("time_constraint") * 60 ## 초단위로 변환 

# 시뮬레이션 시작 시 카메라 데이터 발행 시작 - 카메라 비활성화로 제거
# publish_ros2_camera_data(cam_list)
simulation_app.update()
# set_scoreboard_team_info("Direct Simulation", 1)  # 스코어보드 비활성화
print("[INFO] Simulation started, robot is ready on the map")

while simulation_app.is_running():
    

    simulation_context.step(render=True)

    # competitor_request_message = handler.update_one()  # 대회 프로세스 제거
    competitor_request_message = None
    if competitor_request_message is not None:
        print(f"Message Handling: {competitor_request_message.msg.name}({competitor_request_message.msg.value})")
        if competitor_request_message.msg.value == MessageType.TIME_CONSTRAINT_EXPIRED.value : 
            print(f"  - [Platform->Competitor] Time constraint expired")
            comeptition_status = COMPETITION_STATUS_FINISHING

        elif competitor_request_message.msg.value == MessageType.COMPETITOR_APP_STARTED.value : 
            print(f"  - [Competitor->Platform] Competitor application started")

            payload = competitor_request_message.payload

            print(f"    >     Team name: {payload.team}")
            print(f"    > Applied stage: {payload.stage}")

            # handler.start_competition(payload.team, payload.token, payload.stage)  # 대회 프로세스 제거

            # publish_ros2_camera_data(cam_list)  # 카메라 비활성화로 제거
            simulation_app.update()

            # team_name, apply_stage = handler.get_team_info()  # 대회 프로세스 제거
            team_name, apply_stage = "Direct Simulation", 1
            # set_scoreboard_team_info(team_name, apply_stage)  # 스코어보드 비활성화

            comeptition_status = COMPETITION_STATUS_IN_PROGRESS

        elif competitor_request_message.msg.value == MessageType.REPORT_STAGE1_COMPLETED.value :
            print(f"  - [Competitor->Platform] Stage 1 completed")

            # stage1_score, stage2_score = handler.get_stage_scores()  # 대회 프로세스 제거
            stage1_score, stage2_score = 0, 0
            # set_scoreboard_score_stage1(stage1_score)  # 스코어보드 비활성화
            # set_scoreboard_score_stage2(0.0)  # 스코어보드 비활성화

            print(f"    > Stage 1 score: {stage1_score}")

            if False:  # handler.competitor_apply_stage == 1:  # 대회 프로세스 제거 
                print(f"  - [Competitor->Platform] Competition finished(Competitor applied to stage 1)")
                comeptition_status = COMPETITION_STATUS_FINISHING

        elif competitor_request_message.msg.value == MessageType.REPORT_STAGE2_COMPLETED.value :
            print(f"  - [Competitor->Platform] Stage 2 completed. Competition finished")

            # stage1_score, stage2_score = handler.get_stage_scores()  # 대회 프로세스 제거
            stage1_score, stage2_score = 0, 0
            # set_scoreboard_score_stage2(stage2_score)  # 스코어보드 비활성화

            print(f"    >  Final scores: Stage 1 score: {stage1_score}, Stage 2 score: {stage2_score}")   

            comeptition_status = COMPETITION_STATUS_FINISHING

    # Update scoreboard UI 
    if comeptition_status == COMPETITION_STATUS_FINISHING:
        # time_elapsed = handler.get_time_elapsed()  # 대회 프로세스 제거
        time_elapsed = 0

        # update_scoreboard(time_elapsed, time_constraint)  # 스코어보드 비활성화

        # handler.stop_competition()  # 대회 프로세스 제거
        stop_ros_node()

        comeptition_status = COMPETITION_STATUS_FINISHED

    elif comeptition_status == COMPETITION_STATUS_IN_PROGRESS:
        # time_elapsed = handler.get_time_elapsed()  # 대회 프로세스 제거
        time_elapsed = 0

        # update_scoreboard(time_elapsed, time_constraint)  # 스코어보드 비활성화
    elif comeptition_status == COMPETITION_STATUS_READY:
        # update_scoreboard(None, time_constraint)  # 스코어보드 비활성화
        pass


simulation_context.stop()
simulation_app.close()
