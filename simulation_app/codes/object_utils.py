# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import random
import traceback
from pxr import Gf
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import UsdPhysics, Sdf
import omni.kit.commands
import omni.usd
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.prims import RigidPrim
from omni.kit.commands import execute
from omni.physx.scripts import utils
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from configurations import Configurations
import os
from omni.isaac.debug_draw import _debug_draw
import omni.kit.commands
import yaml

# 카메라가 촬영하는 지면 영역 계산
import math

CONST_MISSION_OBJECT_STAGE_PATH = "/world/trash"
CONST_MISSION_OBJECT_RANDOM_POSITION_OFFSET_RANGE = 2.5



# yaml.Dumper 대신 SafeDumper를 상속받아 커스텀 Dumper 생성
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
    
    # tuple을 list로 변환하는 representer 추가
    def represent_tuple(self, data):
        return self.represent_sequence('tag:yaml.org,2002:seq', list(data))

# tuple representer 등록
NoAliasDumper.add_representer(tuple, NoAliasDumper.represent_tuple)


def get_collapsed_rectangle(mission_area:dict, margin:float) -> list:
    """
    사각형을 margin만큼 안쪽으로 축소했을 때의 꼭지점들을 계산
    중심점으로부터 각 꼭지점까지의 벡터를 margin만큼 줄여서 새로운 꼭지점을 계산
    
    Args:
        mission_area: 사각형 영역의 꼭지점 좌표
            - top_left: [x, y, z]
            - top_right: [x, y, z]
            - bottom_right: [x, y, z]
            - bottom_left: [x, y, z]
        margin: 축소할 거리 (미터 단위)
    
    Returns:
        collapsed_vertices: 축소된 사각형의 꼭지점 좌표 리스트 [top_left, top_right, bottom_right, bottom_left]
            - 각 꼭지점은 [x, y, z] 형태의 numpy array
    """
    # 입력 검증
    required_keys = ["top_left", "top_right", "bottom_right", "bottom_left"]
    if not all(key in mission_area for key in required_keys):
        raise ValueError("mission_area must contain all required vertex keys")
    if margin < 0:
        raise ValueError("margin must be non-negative")
    
    # numpy array로 변환
    vertices = [np.array(mission_area[key]) for key in required_keys]
    tl, tr, br, bl = vertices
    ground_z = tl[2]  # z 좌표는 모든 점에서 동일하다고 가정
    
    try:
        # 중심점 계산
        center = np.mean(vertices, axis=0)
        
        # 각 꼭지점에서 중심점까지의 벡터 계산
        vectors = [v - center for v in vertices]
        
        # 각 벡터의 길이 계산
        lengths = [np.linalg.norm(v[:2]) for v in vectors]
        
        # 각 벡터의 새로운 길이 계산 (length가 margin보다 크면 length-margin, 작으면 length)
        new_lengths = [length - margin if length > margin else length for length in lengths]
        
        # scale factors 계산
        scale_factors = [new_length / length if length > 0 else 0 for new_length, length in zip(new_lengths, lengths)]
        
        # 새로운 꼭지점 계산
        collapsed_vertices = []
        for i, vector in enumerate(vectors):
            # xy 평면에서만 스케일링
            scaled_vector = np.array([
                vector[0] * scale_factors[i],
                vector[1] * scale_factors[i],
                vector[2]  # z 좌표는 유지
            ])
            new_vertex = center + scaled_vector
            collapsed_vertices.append(new_vertex)
                
        return collapsed_vertices
        
    except Exception as e:
        raise ValueError(f"Failed to calculate collapsed rectangle: {str(e)}")         


def draw_mission_area(mission_area):
    """
    미션 영역을 시각화하는 함수
    
    Args:
        mission_area (dict): 미션 영역의 네 모서리 좌표
    """
    # debug draw 인터페이스 가져오기
    draw = _debug_draw.acquire_debug_draw_interface()
    if draw is None:
        return
        
    # 라인 색상 설정 (RGBA)
    color = np.array([1.0, 0.0, 0.0, 1.0])  # 빨간색
    inner_color = np.array([0.0, 0.0, 1.0, 1.0])  # 빨간색
    
    # 네 모서리 점들
    tl = mission_area["top_left"]
    tr = mission_area["top_right"]
    bl = mission_area["bottom_left"]
    br = mission_area["bottom_right"]
    
    collapsed_area = get_collapsed_rectangle(mission_area, 0.8)
    c_tl = collapsed_area[0]
    c_tr = collapsed_area[1]
    c_br = collapsed_area[2]
    c_bl = collapsed_area[3]

    # 라인 그리기
    point_list_1 = [
        tl, tr, br, bl, c_tl, c_tr, c_br, c_bl
    ]
    point_list_2 = [
        tr, br, bl, tl, c_tr, c_br, c_bl, c_tl
    ]
    thickness = 3.0
    
    colors = [color, color, color, color, inner_color, inner_color, inner_color, inner_color]
    thicknesses = [thickness, thickness, thickness, thickness, thickness, thickness, thickness, thickness]
    draw.draw_lines(point_list_1, point_list_2, colors, thicknesses)


# ------------------------------------------------------------
# world에 배치될 모든 object들의 마찰 계수 설정
# ------------------------------------------------------------
def set_friction():
    execute(
        "AddRigidBodyMaterialCommand",
        stage=omni.usd.get_context().get_stage(),
        path="/world/ObjectFriction",
    )
    execute(
        "ChangeProperty",
        prop_path=Sdf.Path("/world/ObjectFriction.physics:dynamicFriction"),
        value=0.4,
        prev=0.0,
        usd_context_name=omni.usd.get_context().get_stage(),
    )
    execute(
        "ChangeProperty",
        prop_path=Sdf.Path("/world/ObjectFriction.physics:staticFriction"),
        value=0.4,
        prev=0.0,
        usd_context_name=omni.usd.get_context().get_stage(),
    )

def get_mission_object_positions(mission_area:dict, n:int, margin_distance:float=0.8):
    """
    사각형 영역 내에 n개의 점을 랜덤하게 분포시키는 함수
    경계선으로부터 margin_distance 안쪽 영역에만 점을 배치
    
    Args:
        mission_area: 사각형 영역의 꼭지점 좌표
            - top_left: [x, y, z]
            - top_right: [x, y, z]
            - bottom_left: [x, y, z]
            - bottom_right: [x, y, z]
        n: 배치할 점의 개수
        margin_distance: 경계선으로부터의 최소 거리 (미터 단위, 기본값 0.2m)
    
    Returns:
        points: n개의 점 좌표 리스트 [[x1,y1,z1], [x2,y2,z2], ...]
    """
    # 원본 사각형의 꼭지점
    vertices = [
        mission_area["top_left"],
        mission_area["top_right"],
        mission_area["bottom_right"],
        mission_area["bottom_left"]
    ]
    ground_z = vertices[0][2] + 0.5  # z 좌표는 모든 점에서 동일해야 함

    def is_point_inside_polygon(point, vertices, epsilon=1e-10):
        """점이 다각형 내부에 있는지 확인 (수치 안정성 개선)"""
        def get_line_side(p, line_p1, line_p2):
            return (line_p2[0] - line_p1[0]) * (p[1] - line_p1[1]) - \
                   (line_p2[1] - line_p1[1]) * (p[0] - line_p1[0])
        
        sides = []
        for i in range(4):
            next_i = (i + 1) % 4
            side = get_line_side(point, vertices[i][:2], vertices[next_i][:2])
            # 경계선 상의 점도 내부로 간주
            if abs(side) < epsilon:
                side = 0
            sides.append(side)
        
        # 모든 부호가 같거나 0이어야 함
        return all(side >= 0 for side in sides) or all(side <= 0 for side in sides)

    def generate_random_point(inner_vertices):
        """축소된 영역 내부의 랜덤한 점 생성"""
        max_attempts = 100
        attempts = 0
        
        while attempts < max_attempts:
            # 축소된 영역의 바운딩 박스 내에서 랜덤한 점 생성
            x = random.uniform(min(v[0] for v in inner_vertices), max(v[0] for v in inner_vertices))
            y = random.uniform(min(v[1] for v in inner_vertices), max(v[1] for v in inner_vertices))
            point = np.array([x, y, ground_z])
            
            if is_point_inside_polygon(point[:2], inner_vertices):
                return point
            
            attempts += 1
        
        # 실패 시 영역의 중심점 반환
        center = np.mean(inner_vertices, axis=0)
        return center

    try:
        # 축소된 사각형 계산
        inner_vertices = get_collapsed_rectangle(mission_area, margin_distance)
        
        # 영역의 중심점과 크기 계산
        center = np.mean(inner_vertices, axis=0)
        dists = [np.linalg.norm(v[:2] - center[:2]) for v in inner_vertices]
        min_dist = min(dists)
        
        # 격자 크기 계산 (최소 거리의 1/4로 설정)
        grid_size = min_dist / (2 * math.sqrt(n))
        
        # 점 생성
        points = []
        max_total_attempts = n * 100  # 전체 최대 시도 횟수
        total_attempts = 0
        
        while len(points) < n and total_attempts < max_total_attempts:
            # 새로운 점 생성
            point = generate_random_point(inner_vertices)
            
            # 다른 점들과의 거리 확인
            if not points or all(np.linalg.norm(point[:2] - np.array(p)[:2]) >= grid_size for p in points):
                points.append(point.tolist())
            
            total_attempts += 1

        # 디버깅: 생성된 모든 점이 축소된 영역 내부에 있는지 확인
        invalid_points = []
        for i, point in enumerate(points):
            if not is_point_inside_polygon(np.array(point)[:2], inner_vertices):
                invalid_points.append((i, point))
        
        if invalid_points:
            for idx, point in invalid_points:
                # 가장 가까운 경계까지의 거리 계산
                min_dist = float('inf')
                for i in range(4):
                    j = (i + 1) % 4
                    v1 = inner_vertices[i][:2]
                    v2 = inner_vertices[j][:2]
                    # 점과 선분 사이의 거리 계산
                    line_vec = v2 - v1
                    point_vec = np.array(point[:2]) - v1
                    line_len = np.linalg.norm(line_vec)
                    line_unit = line_vec / line_len
                    proj_len = np.dot(point_vec, line_unit)
                    proj_len = max(0, min(line_len, proj_len))
                    proj_point = v1 + line_unit * proj_len
                    dist = np.linalg.norm(np.array(point[:2]) - proj_point)
                    min_dist = min(min_dist, dist)
        
        if len(points) < n:
            pass
        
        return points

    except ValueError as e:
        # 에러 발생 시 중심점 주변에 점들을 생성
        center = np.mean(vertices, axis=0)
        points = []
        for _ in range(n):
            angle = random.uniform(0, 2 * np.pi)
            r = random.uniform(0, margin_distance * 0.5)
            point = [
                center[0] + r * np.cos(angle),
                center[1] + r * np.sin(angle),
                ground_z
            ]
            points.append(point)
        return points

def load_answer_sheet(answer_sheet_file_path:str): 
    try:
        with open(answer_sheet_file_path, "r") as f:
            answer_sheet = yaml.safe_load(f)
        return answer_sheet
    except Exception as e:
        raise ValueError(f"Failed to load answer sheet: {str(e)}")


# ------------------------------------------------------------
# 시나리오에 정의된 mission object(쓰레기)들에 대한 usd 파일을 world에 배치하는 함수. DEPRECATED @see publish_objects() 함수와 동일한 기능을 수행함    
def load_or_generate_answer_sheet(configurations:Configurations): 
    ground_truth_output_dir = os.path.join(configurations.get_working_directory_path(), 'scenario-data/answer-sheets')
    answer_sheet_file_path = os.path.join(ground_truth_output_dir, f"{configurations.get_scenario_id()}_answer_sheet.yaml")
    used_mission_object_usds = configurations.get_used_mission_object_usds()

    if configurations.force_generate_answer_sheet():
        try: 
            print(f"[Platform] Loading answer sheet from {answer_sheet_file_path}")
            answer_sheet = load_answer_sheet(answer_sheet_file_path)
            return answer_sheet
        except Exception as e:
            print(f"[ERROR] Warning: Failed to load answer sheet: {str(e)}")
            traceback.print_exc()
            pass

    answer_sheet = {}
    answer_sheet["scenario_id"] = configurations.get_scenario_id()
    answer_sheet["start_position"] = configurations.get_robot_start_point()
    answer_sheet["camera_info"] = []
    answer_sheet["mission_area"] = []
    answer_sheet["mission_objects"] = []

    # 시나리오에서 정의된 모든 mission 정보(CCTV 카메라 별) 조회 
    missions = configurations.get_mission()
    for mission in missions:
        # mission 영역 정보 조회 
        area_name = mission["area_name"]        # 미션 영역 이름 (chungmu1, chungmu2, gunja1, ...)
        camera_info = mission["camera_info"]    # 카메라 정보 (카메라 위치, 카메라 방향, 카메라 확대/축소 비율, ...)


        # configurations에서 카메라 파라미터 추출
        camera_pos = np.array(camera_info["position"])
        # Euler angles 추출 (degree to radian)
        camera_rotation = camera_info["rotation"]
        focal_length = camera_info["focal_length"] * 10
        horizontal_aperture = camera_info["horizontal_aperture"]
        aspect_ratio = 9/16  # 16:9 비율
        
        ground_height = camera_info["ground_height"]
        
        corners = compute_ground_intersections(camera_pos, camera_rotation, focal_length, horizontal_aperture, aspect_ratio, ground_height)
        # 결과 저장
        mission_area = {
            "top_left": corners[0],
            "top_right": corners[1],
            "bottom_right": corners[2],
            "bottom_left": corners[3]
        }

        answer_sheet["mission_area"].append({
            area_name: mission_area
            })

        mission_objects = mission["mission_objects"] # 미션 대상 영역에 배치될 미션 오브젝트 유형별 쓰래기 갯수 (object_name, count)

        # 미션 영역에 배치된 객체의 총 수를 계산 
        number_of_objects = 0
        for mission_object in mission_objects:
            number_of_objects += mission_object["count"]

        mission_object_positions = get_mission_object_positions(mission_area, number_of_objects)
        mission_object_index = 0
        
        # 미션 영역에 배치될 미션 오브젝트 유형별 쓰래기 갯수 정보에 대해 loop을 돌면서 
        for mission_object in mission_objects:
            # 개별 mission object의 이름과 갯수 정보를 조회 
            object_name = mission_object["object_name"]
            count = mission_object["count"]

            # 조회된 갯수 만큼 loop을 돌면서 
            for seq in range(count):

                obj_position = mission_object_positions[mission_object_index]
                mission_object_index += 1

                new_position = (
                    obj_position[0],
                    obj_position[1],
                    obj_position[2],
                )

                # mission object가 publish될 때 회전각도를 random하게 결정되도록 함. 기존에는 config.yaml에 정의하였음
                random_orientation_x = np.random.uniform(-180, 180)
                random_orientation_y = np.random.uniform(-180, 180)
                random_orientation_z = np.random.uniform(-180, 180)
       
                usd_asset = used_mission_object_usds[object_name]

                answer_sheet["mission_objects"].append({
                    "area_name": area_name,
                    "class_name": object_name,
                    "seq": seq,
                    "recyclable": usd_asset["recyclable"],
                    "position": new_position,
                    "rotation": [random_orientation_x, random_orientation_y, random_orientation_z],
                })
        
    # answer_sheet를 YAML 파일로 저장
    os.makedirs(ground_truth_output_dir, exist_ok=True)
    
    scenario_id = answer_sheet["scenario_id"]
    output_file = os.path.join(ground_truth_output_dir, f"{scenario_id}_answer_sheet.yaml")
    
    # 파일이 이미 존재하면 삭제
    if os.path.exists(output_file):
        os.remove(output_file)
        
    # answer_sheet를 YAML 형식으로 저장
    with open(output_file, 'w') as f:
        yaml.dump(answer_sheet, f, Dumper=NoAliasDumper, default_flow_style=False)

    return answer_sheet











# ------------------------------------------------------------
# 시나리오에 정의된 mission object(쓰레기)들에 대한 usd 파일을 world에 배치하는 함수. DEPRECATED @see publish_objects() 함수와 동일한 기능을 수행함    
def deploy_mission_objects(stage:any, configurations:Configurations): 
    result_prime_path_list = []

    resources_dir = configurations.get_resources_directory_path()
    # 미션 오브젝트의 USD 파일이 저장된 경로
    mission_object_asset_path = os.path.join(resources_dir, "assets")

    # 시나리오에서 사용되는 미션 오브젝트의 USD 정보
    used_mission_object_usds = configurations.get_used_mission_object_usds()


    answer_sheet = load_or_generate_answer_sheet(configurations)


    # 시나리오의 고유 ID를 조회: demo, chungmu, gunja, ...
    scenario_id = answer_sheet["scenario_id"]

    mission_areas = answer_sheet["mission_area"]
    mission_objects = answer_sheet["mission_objects"]

    # # mission_areas dict를 올바른 형식으로 순회
    # for area_name, mission_area in mission_areas.items():
    #     draw_mission_area(mission_area)
    # mission_areas 리스트의 각 아이템을 순회

    if configurations.draw_mission_debug_lines():
        for area_dict in mission_areas:
            # 각 딕셔너리에서 첫 번째 (유일한) key-value 쌍을 가져옴
            for area_name, area_data in area_dict.items():
                draw_mission_area(area_data)

    for mission_object in mission_objects:

        area_name = mission_object["area_name"]
        object_name = mission_object["class_name"]
        seq = mission_object["seq"]
        position = mission_object["position"]
        rotation = mission_object["rotation"]

        prim_path = f"{CONST_MISSION_OBJECT_STAGE_PATH}/{area_name}_{object_name}_{seq}"
        usd_asset = used_mission_object_usds[object_name]

        # 회전 각도를 쿼터니언으로 변환 
        quats_orientation = euler_angles_to_quats(np.array([rotation[0], rotation[1], rotation[2]]), degrees=True, extrinsic=False)

        # 미션 오브젝트의 USD 파일 경로 조회 
        mission_object_usd_path = os.path.join(mission_object_asset_path, used_mission_object_usds[object_name]["usd_file"])
        
        # IsaacSimSpawnPrim 명령어 실행으로 world에 mission object를 배치 
        execute(
            "IsaacSimSpawnPrim",
            usd_path=mission_object_usd_path,
            prim_path=prim_path,
            translation=position,
            rotation=(
                quats_orientation[3],
                quats_orientation[0],
                quats_orientation[1],
                quats_orientation[2],
            ),
        )
        
        # 배치된 mission object의 USD 프라임 조회
        usd_stage = stage.get_current_stage()
        usd_prime = usd_stage.GetPrimAtPath(prim_path)

        # 배치된 mission object의 물리 속성 설정
        utils.setRigidBody(usd_prime, "convexDecomposition", False)

        # 배치된 mission object의 시멘틱 레이블 설정
        label = f"{object_name}"
        add_update_semantics(usd_prime, semantic_label=label, type_label="class")

        result_prime_path_list.append(prim_path)
  
    return result_prime_path_list, answer_sheet


# stage:any, configurations:Configurations, resources_dir:str,ground_truth_output_dir:str
def set_mission_object_semantics_attributes(configurations: Configurations):

    # 시나리오의 고유 ID를 조회: demo, chungmu, gunja, ...
    scenario_id = configurations.get_scenario_id()

    # 시나리오에서 사용되는 미션 오브젝트의 USD 정보
    used_mission_object_usds = configurations.get_used_mission_object_usds()

    # 시나리오에서 정의된 모든 mission 정보(CCTV 카메라 별) 조회 
    missions = configurations.get_mission()
    for mission in missions:
        # mission 영역 정보 조회 
        area_name = mission["area_name"]        # 미션 영역 이름 (chungmu1, chungmu2, gunja1, ...)
        camera_info = mission["camera_info"]    # 카메라 정보 (카메라 위치, 카메라 방향, 카메라 확대/축소 비율, ...)
        mission_objects = mission["mission_objects"] # 미션 대상 영역에 배치될 미션 오브젝트 유형별 쓰래기 갯수 (object_name, count)

        # 미션 영역에 배치될 미션 오브젝트 유형별 쓰래기 갯수 정보에 대해 loop을 돌면서 
        for mission_object in mission_objects:
            # 개별 mission object의 이름과 갯수 정보를 조회 
            object_name = mission_object["object_name"]
            count = mission_object["count"]

            # 조회된 갯수 만큼 loop을 돌면서 
            for seq in range(count):
                # prime path 지정: /world/trash/...
                prim_path = f"{CONST_MISSION_OBJECT_STAGE_PATH}/{area_name}_{object_name}_{seq}"
                # 개별 mission object의 USD 정보 조회 {type: ycb, usd_file: 002_master_chef_can.usd, scale: [0.4, 0.4, 0.4]}
                usd_asset = used_mission_object_usds[object_name]
                
                usd_prim = get_prim_at_path(prim_path)

                execute(
                    "IsaacSimScalePrim",
                    prim_path=prim_path,
                    scale=usd_asset["scale"],
                )
                execute(
                    "BindMaterial",
                    material_path="/world/ObjectFriction",
                    prim_path=prim_path,
                    strength=["weakerThanDescendants"],
                    material_purpose="",
                )
                utils.setRigidBody(usd_prim, "convexDecomposition", False)
                label = f"{object_name}"
                add_update_semantics(usd_prim, semantic_label=label, type_label="class")


def set_mission_object_physics(mission_objects_prim_paths: list):

    for prim_path in mission_objects_prim_paths:
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverPositionIterationCount"),
            value=60,
            prev=16,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverVelocityIterationCount"),
            value=60,
            prev=1,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:maxDepenetrationVelocity"),
            value=20.0,
            prev=3.0,
            usd_context_name=omni.usd.get_context().get_stage(),
        )

############ deprecated functions START############


## deprecated @see publish_mission_scenario
def publish_objects(asset_attribute, ground_truth_output_dir, stage, robot_start_point, standard_pos, object_list):
    ## gt 생성용 ##
    output_file = os.path.join(ground_truth_output_dir, "publish_issacsim.txt")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    if not os.path.exists(output_file):
        open(output_file, "w").close()
    else:
        os.remove(output_file)

    prim_paths = []
    offset_range = 3

    with open(output_file, "w") as f:
        f.write(f"StartPoint,{robot_start_point[0]},{robot_start_point[1]},{robot_start_point[2]}\n")

        for building in standard_pos:
            for obj in object_list:
                prim_path = f"/world/trash/"+building+"_"+obj
                asset = asset_attribute[obj]
                usd_stage = stage.get_current_stage()
                offset_x = np.random.uniform(-offset_range, offset_range)
                offset_y = np.random.uniform(-offset_range, offset_range)
                obj_rand_x = standard_pos[building]["translation"][0] + offset_x
                obj_rand_y = standard_pos[building]["translation"][1] + offset_y
                obj_z = standard_pos[building]["translation"][2]

                new_position = (
                    obj_rand_x,
                    obj_rand_y,
                    obj_z,
                )
                quats_orientation = euler_angles_to_quats(np.array(asset["orientation"]), degrees=True, extrinsic=False)

                home_dir = os.getcwd()
                usd_path = os.path.join(asset_attribute[obj]["usd_dir"], asset_attribute[obj]["usd_file"])
                usd_path = os.path.join(home_dir, usd_path)

                execute(
                    "IsaacSimSpawnPrim",
                    usd_path=usd_path,
                    prim_path=prim_path,
                    translation=new_position,
                    rotation=(
                        quats_orientation[3],
                        quats_orientation[0],
                        quats_orientation[1],
                        quats_orientation[2],
                    ),
                )
                usd_prim = usd_stage.GetPrimAtPath(prim_path)
                utils.setRigidBody(usd_prim, "convexDecomposition", False)
                label = f"{obj}"
                add_update_semantics(usd_prim, semantic_label=label, type_label="class")
                prim_paths.append((prim_path, standard_pos[building], obj))
                if prim_path.startswith("/world/trash/Chungmu"):
                    if not prim_path.endswith("wood_block") and not prim_path.endswith("mug"):
                        f.write(f"{prim_path},{obj_rand_x},{obj_rand_y},{obj_z}\n")


    return prim_paths

## deprecated @see set_mission_object_semantics_attributes
def set_object_attributes_semantics(asset_attribute, standard_pos, object_list):
    for building in standard_pos:
        for obj in object_list:
            prim_path = f"/world/trash/"+building+"_"+obj
            prim = get_prim_at_path(prim_path)
            execute(
                "IsaacSimScalePrim",
                prim_path=prim_path,
                scale=asset_attribute[obj]["scale"],
            )
            execute(
                "BindMaterial",
                material_path="/world/ObjectFriction",
                prim_path=prim_path,
                strength=["weakerThanDescendants"],
                material_purpose="",
            )
        utils.setRigidBody(prim, "convexDecomposition", False)
        label = f"{obj}"
        add_update_semantics(prim, semantic_label=label, type_label="class")



def compute_ground_intersections(position, rotation_euler, focal_length, horizontal_aperture, aspect_ratio=16/9, ground_height=0.0):
    """
    지면과의 교차점을 계산합니다.
    
    Args:
        position: 카메라 위치 (numpy array)
        rotation_euler: 카메라 회전 각도 (roll, pitch, yaw) in degrees, XYZ 순서의 로컬 축 회전
        focal_length: 초점 거리 (mm)
        horizontal_aperture: 수평 조리개 크기 (mm)
        aspect_ratio: 화면 비율 (기본값: 16/9)
        ground_height: 지면 높이 (기본값: 0.0)
        
    Returns:
        numpy array: 교차점 좌표들 (4x3) 또는 None
    """

    # Step 2: Camera world pose
    cam_pos = np.array(position)
    # XYZ (Roll → Pitch → Yaw) 순서로 로컬 축 회전 적용
    cam_rot = R.from_euler('zyx', rotation_euler[::-1], degrees=True).as_matrix()

    # Step 3: Rays in camera local space (normalized)
    # 카메라 좌표계: z가 전방, y가 위쪽, x가 오른쪽 (Z-up 월드 좌표계 기준)
    x = (horizontal_aperture / 2)   # mm
    y = (x * aspect_ratio)          # mm
    z = focal_length

    rays_cam = np.array([
        [-x,  y, -z],  # TL
        [ x,  y, -z],  # TR
        [ x, -y, -z],  # BR
        [-x, -y, -z],  # BL
    ])
    rays_cam = rays_cam / np.linalg.norm(rays_cam, axis=1)[:, np.newaxis]

    # Step 4: Transform rays to world space
    rays_world = rays_cam @ cam_rot.T

    # Step 5: Ray-plane intersection (ground plane at z=ground_height)
    ground_normal = np.array([0, 0, 1])
    plane_point = np.array([0, 0, ground_height])

    # 모든 교차점을 저장할 배열 (4x3)
    ground_points = np.zeros((4, 3))
    valid_mask = np.zeros(4, dtype=bool)

    for i, dir_world in enumerate(rays_world):
        denom = np.dot(ground_normal, dir_world)

        if np.abs(denom) < 1e-6:
            continue

        t = np.dot(ground_normal, (plane_point - cam_pos)) / denom
        
        if t < 0:
            continue
            
        point = cam_pos + t * dir_world
        ground_points[i] = point
        valid_mask[i] = True

    if valid_mask.sum() < 4:
        return None
        
    # numpy 배열을 일반 float 배열로 변환
    return ground_points.astype(float).tolist()

def get_mission_area_coordinates(configurations:Configurations):
    """
    카메라의 투영 영역을 계산합니다.
    
    Args:
        configurations: 시뮬레이션 설정 객체
        
    Returns:
        mission_area: 투영 영역의 코너점들을 포함하는 딕셔너리
    """
    missions = configurations.get_mission()
    mission_areas = {}
    
    for mission in missions:
        area_name = mission["area_name"]
        camera_info = mission["camera_info"]
        
        # configurations에서 카메라 파라미터 추출
        camera_pos = np.array(camera_info["position"])
        ground_height = camera_info["ground_height"]
        
        # Euler angles 추출 (degree to radian)
        camera_rotation = camera_info["rotation"]
        
        focal_length = camera_info["focal_length"] * 10
        horizontal_aperture = camera_info["horizontal_aperture"]
        aspect_ratio = 9/16  # 16:9 비율
        
        corners = compute_ground_intersections(camera_pos, camera_rotation, focal_length, horizontal_aperture, aspect_ratio, ground_height)

        # 결과 저장 (이미 올바른 순서로 계산됨)
        mission_area = {
            "top_left": corners[0],
            "top_right": corners[1],
            "bottom_right": corners[2],
            "bottom_left": corners[3]
        }
        
        mission_areas[area_name] = mission_area
    
    return mission_areas 

## deprecated @see set_mission_object_physics   
def set_object_physics(prim_paths):
    for prim_path, _, _ in prim_paths:
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverPositionIterationCount"),
            value=60,
            prev=16,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverVelocityIterationCount"),
            value=60,
            prev=1,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:maxDepenetrationVelocity"),
            value=20.0,
            prev=3.0,
            usd_context_name=omni.usd.get_context().get_stage(),
        )

############ deprecated functions END ############
