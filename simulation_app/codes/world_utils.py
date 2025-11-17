# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import numpy as np
from omni.isaac.core.utils import viewports, stage
from omni.kit.commands import execute
import carb
from pxr import UsdPhysics, UsdGeom, Sdf, Gf
import omni.graph.core as og
import omni.usd
import usdrt

CONST_WORLD_STAGE_PATH = "/world/background"


def is_valid_mesh_for_collision(prim):
    """메시가 충돌 처리를 위한 유효한 지오메트리를 가지고 있는지 확인"""
    if not prim.IsA(UsdGeom.Mesh):
        return False
    
    mesh = UsdGeom.Mesh(prim)
    
    try:
        # 포인트 데이터 확인 - 캐시된 값 사용
        points = mesh.GetPointsAttr().Get(time=0)
        if not points or len(points) == 0:
            carb.log_warn(f"[Mesh Validation] No points found for {prim.GetPath()}")
            return False
        
        # 면(face) 데이터 확인 - 캐시된 값 사용
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get(time=0)
        if not face_vertex_counts or len(face_vertex_counts) == 0:
            carb.log_warn(f"[Mesh Validation] No face vertex counts found for {prim.GetPath()}")
            return False
        
        # 정점 인덱스 확인 - 캐시된 값 사용
        face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get(time=0)
        if not face_vertex_indices or len(face_vertex_indices) == 0:
            carb.log_warn(f"[Mesh Validation] No face vertex indices found for {prim.GetPath()}")
            return False
        
        # 메시 크기 확인
        extent = mesh.ComputeExtent()
        if extent.GetSize().GetLength() < 0.001:  # 1mm 미만의 메시는 무시
            carb.log_warn(f"[Mesh Validation] Mesh too small for {prim.GetPath()}")
            return False
        
        # 메시 복잡도 확인 - 임계값 조정
        if len(points) > 50000 or len(face_vertex_indices) > 150000:  # 임계값 증가
            carb.log_warn(f"[Mesh Validation] Mesh too complex for {prim.GetPath()}")
            return False
        
        return True
    except Exception as e:
        carb.log_error(f"[Mesh Validation] Error validating mesh {prim.GetPath()}: {str(e)}")
        return False


def is_top_level_mesh(prim):
    """최상위 메시 프림인지 확인"""
    if not prim.IsA(UsdGeom.Mesh):
        return False
    
    # 부모가 메시인 경우 최상위 메시가 아님
    parent = prim.GetParent()
    if parent and parent.IsA(UsdGeom.Mesh):
        return False
    
    return True


def setup_world_stage_and_viewport(camera_view_eye, camera_view_target, world_usd_path):
    # 카메라 위치 설정
    carmera_view_eye = np.array(camera_view_eye)
    carmera_view_target = np.array(camera_view_target)
    viewports.set_camera_view(eye=carmera_view_eye, target=carmera_view_target)
    carb.log_info(f"[World Setup] Camera view set: eye={carmera_view_eye}, target={carmera_view_target}")

    # 배경 환경 로드
    stage.add_reference_to_stage(world_usd_path, CONST_WORLD_STAGE_PATH)
    carb.log_info(f"[World Setup] Background stage loaded from: {world_usd_path}")

    # CollisionAPI 설정
    usd_stage = omni.usd.get_context().get_stage()
    stage_prims = usd_stage.GetPrimAtPath(CONST_WORLD_STAGE_PATH)
    
    # 모든 하위 프림에 대해 CollisionAPI 적용
    for prim in stage_prims.GetAllChildren():
        try:
            # 최상위 메시에만 CollisionAPI 적용
            if is_top_level_mesh(prim):
                if is_valid_mesh_for_collision(prim):
                    # 기존 CollisionAPI 제거
                    if prim.HasAPI(UsdPhysics.CollisionAPI):
                        prim.RemoveAPI(UsdPhysics.CollisionAPI)
                    
                    # 새로운 CollisionAPI 적용
                    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
                    collision_api.GetCollisionEnabledAttr().Set(True)
                    
                    # 물리 속성 최적화
                    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI(prim)
                    
                    rigid_api.GetMassAttr().Set(1.0)
                    rigid_api.GetInertiaAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
                    rigid_api.GetLinearDampingAttr().Set(0.1)
                    rigid_api.GetAngularDampingAttr().Set(0.1)
                    
                    carb.log_info(f"[World Setup] Successfully applied CollisionAPI to {prim.GetPath()}")
                else:
                    carb.log_warn(f"[World Setup] Invalid mesh for collision at {prim.GetPath()}")
        except Exception as e:
            carb.log_error(f"[World Setup] Error processing prim {prim.GetPath()}: {str(e)}")
            continue

    # 특정 Prim 스케일 조정
    # execute(
    #     "IsaacSimScalePrim",
    #     prim_path="/World/metasejong",
    #     scale=(0.01, 0.01, 0.01),
    # )
    # carb.log_info("[World Setup] /World/metasejong scaled to (0.01, 0.01, 0.01)")

def init_world_stage(): 
    usd_stage = omni.usd.get_context().get_stage()
    stage_prims = usd_stage.GetPrimAtPath(CONST_WORLD_STAGE_PATH)
    
    # Transform 설정
    xform = UsdGeom.Xformable(stage_prims)
    xform.AddTranslateOp().Set((0.0, 0.0, 0.0))
    xform.AddScaleOp().Set((0.01, 0.01, 0.01))

    # 조명 설정
    dome_light = usd_stage.DefinePrim("/world/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)


def setup_score_scope():
    #! 점수 확인용 Scope 생성
    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Score/recycling",
        attributes={},
        select_new_prim=True,
    )

    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Score/trash_pose_estimations",
        attributes={},
        select_new_prim=True,
    )

    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Score/trash",
        attributes={},
        select_new_prim=True,
    )
