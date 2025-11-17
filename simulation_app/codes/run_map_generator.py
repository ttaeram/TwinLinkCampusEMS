from isaacsim import SimulationApp
from matplotlib.path import Path
from scipy.spatial import ConvexHull

ISSAC_INIT_CONFIG = {
    "renderer": "RayTracedLighting", 
    "headless": False,          # GUI 모드로 실행
    "width": 1920,             # 화면 너비
    "height": 1080,            # 화면 높이
    "window_width": 1920,      # 창 너비
    "window_height": 1080,     # 창 높이
    "display_options": 3074    # FULLSCREEN | WINDOW_HIDE_MENU | WINDOW_HIDE_TOOLBAR
}

simulation_app = SimulationApp(ISSAC_INIT_CONFIG)

# viewport 초기화를 위한 update
simulation_app.update()

# viewport가 완전히 초기화될 때까지 대기
import time
time.sleep(1.0)  # 1초 대기

import omni.usd
from omni.usd import get_context
import numpy as np
from scipy.spatial import ConvexHull
import shapely.geometry as sg
import shapely.ops as so
from omni.isaac.core.utils import stage
from pxr import UsdGeom, UsdPhysics
import alphashape


CONST_WORLD_STAGE_PATH = "/world/background"


usd_path = "/home/aisl/data/git/metasejong/metacom2025/resources/models/playground/S3-Dongcheon/SejongUniv_S3.usd"
mash_path = f"{CONST_WORLD_STAGE_PATH}/S3/S3_Site/S3_CarRoad/S3_CarRoad/Section1"


stage.add_reference_to_stage(usd_path, CONST_WORLD_STAGE_PATH)
stage = omni.usd.get_context().get_stage()

# PhysicsBodyAPI 설정 전에 확인
def setup_physics_body(prim):
    if not prim.HasAPI(UsdPhysics.PhysicsBodyAPI):
        physics_body = UsdPhysics.PhysicsBodyAPI.Apply(prim)
        physics_body.CreateRigidBodyEnabledAttr().Set(True)
        return True
    return False

# Step 2: 도로 prim path 목록
road_paths = [
    mash_path
]

# 결과 polygon 리스트
polygon_list = []

for path in road_paths:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"유효하지 않은 prim 경로: {path}")
        continue

    # trigger prim 유효성 검증
    if not prim.IsA(UsdGeom.Mesh):
        print(f"경고: {path}는 Mesh 타입이 아닙니다.")
        continue

    # PhysicsBodyAPI 설정
    if setup_physics_body(prim):
        print(f"PhysicsBodyAPI가 {path}에 설정되었습니다.")
    else:
        print(f"PhysicsBodyAPI가 이미 {path}에 정의되어 있습니다.")

    mesh = UsdGeom.Mesh(prim)
    points_attr = mesh.GetPointsAttr()
    points = points_attr.Get()
    if not points:
        print(f"경고: {path}의 점 정보가 없습니다.")
        continue

    # numpy 변환 후 XY 평면 투영
    points_np = np.array([[p[0], p[1], p[2]] for p in points])
    points_xy = points_np[:, :2]

    # Alpha Shape로 경계 추출
    try:
        print(f"points_xy: {points_xy}")
        # alpha 값은 데이터에 따라 조정 필요 (작을수록 더 복잡한 경계)
        alpha = 1000
        alpha_shape = alphashape.alphashape(points_xy, alpha)

        print(f"alpha_shape: {alpha_shape}")

        if isinstance(alpha_shape, sg.Polygon):
            polygon_list.append(alpha_shape)
        else:
            print(f"Alpha Shape가 폴리곤이 아닙니다 ({path})")
    except Exception as e:
        print(f"Alpha Shape 계산 오류 ({path}): {e}")

# (선택) 전체 도로의 경계를 하나로 합치기
if polygon_list:
    merged = so.unary_union(polygon_list)
    print("병합된 도로 경계 (WKT):", merged.wkt)

# SVG 파일로 저장
if polygon_list:
    from svgwrite import Drawing
    import os

    # SVG 파일 생성
    svg_path = os.path.join(os.path.dirname(__file__), "road_boundary.svg")
    dwg = Drawing(svg_path, profile='tiny')
    
    # 모든 좌표를 양수로 변환하기 위한 오프셋 계산
    min_x = min(min(x for x, y in merged.exterior.coords), 0)
    min_y = min(min(y for x, y in merged.exterior.coords), 0)
    offset_x = abs(min_x) + 100  # 여백 추가
    offset_y = abs(min_y) + 100  # 여백 추가
    
    # 다각형 경계를 SVG path로 변환 (양수 좌표로 변환)
    path_data = f"M {merged.exterior.coords[0][0] + offset_x},{merged.exterior.coords[0][1] + offset_y}"
    for x, y in merged.exterior.coords[1:]:
        path_data += f" L {x + offset_x},{y + offset_y}"
    path_data += " Z"  # 경로 닫기
    
    # SVG에 경로 추가 
    dwg.add(dwg.path(d=path_data, stroke='black', fill='none'))
    
    # SVG 파일 저장
    dwg.save()
    print(f"도로 경계 SVG 파일이 저장되었습니다: {svg_path}")
    print(f"좌표 오프셋: x={offset_x}, y={offset_y}")



















# from isaacsim import SimulationApp
# from matplotlib.path import Path
# from scipy.spatial import ConvexHull

# ISSAC_INIT_CONFIG = {
#     "renderer": "RayTracedLighting", 
#     "headless": True,          # GUI 모드로 실행
#     "width": 1920,             # 화면 너비
#     "height": 1080,            # 화면 높이
#     "window_width": 1920,      # 창 너비
#     "window_height": 1080,     # 창 높이
#     "display_options": 3074    # FULLSCREEN | WINDOW_HIDE_MENU | WINDOW_HIDE_TOOLBAR
# }

# simulation_app = SimulationApp(ISSAC_INIT_CONFIG)

# # viewport 초기화를 위한 update
# simulation_app.update()

# import omni.usd
# from pxr import UsdGeom
# import numpy as np
# from PIL import Image
# from omni.isaac.core.utils import stage
# CONST_WORLD_STAGE_PATH = "/world/background"


# usd_path = "/home/aisl/data/git/metasejong/metacom2025/resources/models/playground/S3-Dongcheon/SejongUniv_S3.usd"
# grid_resolution = 0.05  # 5cm per pixel
# grid_width = 1000
# grid_height = 1000

# # 도로 Prim Path 목록
# road_mesh_prim_paths = [
#     "/world/background/S3/S3_Site/S3_CarRoad/S3_CarRoad/Section1",
# ]

# # Occupancy Grid 초기화 (default: Occupied 100)
# occupancy_grid = np.ones((grid_height, grid_width), dtype=np.uint8) * 100

# # 스테이지 열기
# # omni.usd.get_context().open_stage(usd_path)
# stage.add_reference_to_stage(usd_path, CONST_WORLD_STAGE_PATH)

# stage = omni.usd.get_context().get_stage()

# grid_origin_x = -grid_width * grid_resolution / 2
# grid_origin_y = -grid_height * grid_resolution / 2

# def get_mesh_2d_polygon(prim):
#     mesh = UsdGeom.Mesh(prim)
#     if not mesh:
#         print(f"경고: {prim} prim을 찾을 수 없습니다.")
#         return None
#     points = mesh.GetPointsAttr().Get()
#     if not points:
#         print(f"경고: {prim} prim의 점 정보가 없습니다.")
#         return None
#     # 월드 변환 적용
#     xform = UsdGeom.Xformable(prim)
#     mat = xform.ComputeLocalToWorldTransform(0.0)
#     poly_2d = []
#     for p in points:
#         world_p = mat.Transform(p)
#         poly_2d.append((world_p[0], world_p[1]))
#     return np.array(poly_2d)

# def get_geomsubset_polygons(mesh_prim, subset_name):
#     mesh = UsdGeom.Mesh(mesh_prim)
#     print(f"mesh_prim: {mesh_prim}")
#     points = mesh.GetPointsAttr().Get()
#     face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
#     face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

#     # GeomSubset prim 찾기
#     subset_prim = mesh_prim.GetChild(subset_name)
#     print(f"subset_prim: {subset_prim}")
#     subset = UsdGeom.Subset(subset_prim)
#     face_indices = subset.GetIndicesAttr().Get()

#     # face별로 vertex index 리스트 만들기
#     faces = []
#     idx = 0
#     for count in face_vertex_counts:
#         faces.append(face_vertex_indices[idx:idx+count])
#         idx += count

#     # Section3에 해당하는 face들의 vertex 좌표만 추출
#     polygons = []
#     for face_idx in face_indices:
#         poly = []
#         for vi in faces[face_idx]:
#             v = points[vi]
#             poly.append((v[0], v[1]))  # 2D 투영
#         polygons.append(np.array(poly))
#     return polygons

# def get_geomsubset_hull_path(mesh_prim, subset_name):
#     mesh = UsdGeom.Mesh(mesh_prim)
#     points = mesh.GetPointsAttr().Get()
#     face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
#     face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

#     subset_prim = mesh_prim.GetChild(subset_name)
#     subset = UsdGeom.Subset(subset_prim)
#     face_indices = subset.GetIndicesAttr().Get()

#     faces = []
#     idx = 0
#     for count in face_vertex_counts:
#         faces.append(face_vertex_indices[idx:idx+count])
#         idx += count

#     # Section3에 해당하는 모든 vertex 좌표 모으기
#     all_points = []
#     for face_idx in face_indices:
#         for vi in faces[face_idx]:
#             v = points[vi]
#             all_points.append([v[0], v[1]])
#     all_points = np.array(all_points)

#     # Convex Hull로 외곽선 추출
#     if len(all_points) < 3:
#         return None
#     hull = ConvexHull(all_points)
#     hull_points = all_points[hull.vertices]
#     return Path(hull_points)


# def get_mesh_border_path(mesh_prim):
#     mesh = UsdGeom.Mesh(mesh_prim)
#     points = mesh.GetPointsAttr().Get()
#     face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
#     face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

#     # 모든 vertex 좌표를 2D로 변환
#     all_points = []
#     for p in points:
#         all_points.append([p[0], p[1]])  # x, y 좌표만 사용
#     all_points = np.array(all_points)

#     # Convex Hull로 외곽선 추출
#     if len(all_points) < 3:
#         return None
#     # hull = ConvexHull(all_points)
#     # hull_points = all_points[hull.vertices]
#     return Path(all_points)



# # road prim들의 2D 폴리곤 리스트 생성
# road_polygons = []
# for path in road_mesh_prim_paths:
#     prim = stage.GetPrimAtPath(path)
#     if not prim or not prim.IsValid():
#         print(f"경고: {path} prim을 찾을 수 없습니다.")
#         continue

#     mesh_2d_polygon = get_mesh_2d_polygon(prim)
#     print(f"mesh_2d_polygon: {mesh_2d_polygon}")

#     road_polygons = [mesh_2d_polygon] if mesh_2d_polygon is not None else []


# print(f"road_polygons: {road_polygons}")


# # Occupancy Grid 생성
# for iy in range(grid_height):
#     for ix in range(grid_width):
#         x = grid_origin_x + ix * grid_resolution + grid_resolution / 2
#         y = grid_origin_y + iy * grid_resolution + grid_resolution / 2
#         cell_center = (x, y)
#         is_free = False
#         for poly in road_polygons:
#             if poly.contains_point(cell_center):
#                 is_free = True
#                 break
#         occupancy_grid[iy, ix] = 0 if is_free else 100

# def save_occupancy_map_and_log(occupancy_grid, pgm_path, yaml_path, resolution, origin):
#     # 1. PGM 파일 저장
#     height, width = occupancy_grid.shape
#     with open(pgm_path, 'wb') as f:
#         f.write(b'P5\n')
#         f.write(f'{width} {height}\n255\n'.encode())
#         # occupancy_grid: 0=free, 100=occupied, 205=unknown
#         # PGM: 0=black(occupied), 254=white(free)
#         pgm_data = np.where(occupancy_grid == 0, 254, 0).astype(np.uint8)
#         f.write(pgm_data.tobytes())

#     # 2. YAML 파일 저장
#     yaml_content = f"""image: {pgm_path}
# resolution: {resolution}
# origin: [{origin[0]}, {origin[1]}, {origin[2]}]
# negate: 0
# occupied_thresh: 0.65
# free_thresh: 0.196
# """
#     with open(yaml_path, 'w') as f:
#         f.write(yaml_content)

#     # 3. 비율 로그 출력
#     total = occupancy_grid.size
#     free = np.sum(occupancy_grid == 0)
#     occupied = np.sum(occupancy_grid == 100)
#     unknown = np.sum(occupancy_grid == 205)
#     print(f"[Occupancy Map] 전체 cell: {total}, free: {free}, occupied: {occupied}, unknown: {unknown}")
#     print(f"[Occupancy Map] free 비율: {free/total:.2%}, occupied 비율: {occupied/total:.2%}, unknown 비율: {unknown/total:.2%}")

# # 예시 사용
# # occupancy_grid = ... # (height, width) numpy array, 0=free, 100=occupied, 205=unknown
# # save_occupancy_map_and_log(occupancy_grid, "map.pgm", "map.yaml", 0.05, [0.0, 0.0, 0.0])

# def save_occupancy_map_image(occupancy_grid, img_path, road_polygons=None):
#     # 0: free(흰색), 100: occupied(검정), 205: unknown(회색)
#     color_map = {
#         0:   [255, 255, 255],  # free: white
#         100: [0, 0, 0],        # occupied: black
#         205: [128, 128, 128],  # unknown: gray
#     }
#     h, w = occupancy_grid.shape
#     img = np.zeros((h, w, 3), dtype=np.uint8)
#     for val, color in color_map.items():
#         img[occupancy_grid == val] = color

#     # road_polygons 시각화 (빨간색)
#     if road_polygons is not None:
#         import cv2
#         for poly in road_polygons:
#             # Path 객체에서 polygon 좌표 추출
#             verts = np.array(poly.vertices, dtype=np.float32)
#             # grid 좌표계로 변환
#             px = ((verts[:, 0] - grid_origin_x) / grid_resolution).astype(np.int32)
#             py = ((verts[:, 1] - grid_origin_y) / grid_resolution).astype(np.int32)
#             pts = np.stack([px, py], axis=1)
#             pts = pts.reshape((-1, 1, 2))
#             cv2.polylines(img, [pts], isClosed=True, color=(255, 0, 0), thickness=1)  # 빨간색

#     Image.fromarray(img).save(img_path)
#     print(f"[Occupancy Map] 컬러 이미지 저장 완료: {img_path}")

# # 예시 사용
# # save_occupancy_map_image(occupancy_grid, "demo_map.png")

# save_occupancy_map_and_log(occupancy_grid, "demo_map.pgm", "demo_map.yaml", grid_resolution, [grid_origin_x, grid_origin_y, 0.0])
# save_occupancy_map_image(occupancy_grid, "demo_map.png", road_polygons=road_polygons)

