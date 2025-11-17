# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#


# Configurations 정의를 포함하여 모든 설정 파일을 로드하는 클래스
# 모든 설정 파일은 config 폴더에 있어야 함
# 구동 환경(local, docker)에 따라 달라지는 설정 및 상수 값들은 config.yaml 파일로 정의
# 경진대회 훈련/경연 시나리오에 따라 달라지는 설정은 /scenario-data/{scenario_id}.yaml 파일로 정의 


import os
import yaml

from common_utils import check_missing_mandatory_key

class Configurations:
    _instance = None
    _initialized = False
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Configurations, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        # 이미 초기화된 경우 skip
        if Configurations._initialized:
            return

        """
        환경설정 (config.yaml) 및 시나리오 설정 (scenario_{scenario_id}.yaml) 파일을 로드하는 객체를 초기화합니다.
        
        Args:
            working_directory (str): 시뮬레이션 앱 실행 경로. local 환경은 {project_root}, docker 환경은 /simulation_app
            docker_environment (str): Docker 환경에서 실행했는지 여부. YES = Docker 환경, not YES = Local 운용 환경
            scenario_id (str): 시나리오 파일 이름 (확장자 제외)
            
        Raises:
            FileNotFoundError: 시나리오 파일이 존재하지 않는 경우
            ValueError: YAML 파일 파싱 오류, 또는 시나리오 정의에서 지원되지 않는 미션 오브젝트 유형이 사용된 경우
            KeyError: YAML 파일 내용에 필수 키가 누락된 경우
            RuntimeError: 기타 파일 로드 오류
        """

        self.working_directory = os.getcwd()
        docker_environment = os.environ.get("ENV_METASEJONG_DOCKER")
        scenario_id = os.environ.get("ENV_METASEJONG_SCENARIO", 'demo')

        if docker_environment == "YES":
            self.working_directory = '/metacom2025'
            self.runtime_data_directory = os.path.join('/root', '.metasejong')
        else :
            self.working_directory = os.getcwd()
            self.runtime_data_directory = os.path.join(self.working_directory, '.runtime-data')

        CONFIG_DIRECTORY_NAME = "config"
        SCENARIO_SUB_DIRECTORY_NAME = "scenario"

        self.scenario_id = scenario_id
        
        self.config_file_path = os.path.join(self.working_directory, 'simulation_app', CONFIG_DIRECTORY_NAME, f"config.yaml")
        self.scenario_file_path = os.path.join(self.working_directory, 'scenario-data', f"{scenario_id}.yaml")
        self.resources_directory = os.path.join(self.working_directory, 'resources')

        # 로드하고자 하는 설정 파일과 시나리오 파일이 있는지 먼저 확인하고 
        if not os.path.exists(self.config_file_path):
            raise FileNotFoundError(f"Configuration file not found: {self.config_file_path}")
        
        if not os.path.exists(self.scenario_file_path):
            raise FileNotFoundError(f"Scenario description file not found: {self.scenario_file_path}")
            
        # 각각에 대해 YAML 파일 로딩 
        try:
            with open(self.config_file_path, "r") as f:
                self._config_data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"Error while parsing configuration file: {str(e)}")
        except Exception as e:
            raise RuntimeError(f"Error while loading configuration file: {str(e)}")

        try:
            with open(self.scenario_file_path, "r") as f:
                self._scenario_data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"Error while parsing scenario description file: {str(e)}")
        except Exception as e:
            raise RuntimeError(f"Error while loading scenario description file: {str(e)}")

    def force_generate_answer_sheet(self) -> bool:
        """
        답안지를 강제로 생성할지 여부를 반환합니다.
        """
        return self._scenario_data.get("force_generate_answer_sheet", False)

    def draw_mission_debug_lines(self) -> bool:
        """
        미션 디버그 선을 그릴지 여부를 반환합니다.
        """
        return self._scenario_data.get("draw_mission_debug_lines", False)

    def get_working_directory_path(self) -> str:
        """
        작업 디렉토리를 반환합니다.
        """
        return self.working_directory

    def get_robot_usd_path(self, usd_file_name: str) -> str:
        """
        로봇 USD 파일의 이름을 받아서, 운용 환경에 따른 USD 파일의 full path를 반환
        """
        return os.path.join(self.resources_directory, 'models', 'robot', usd_file_name)

    def get_mission_object_usd_path(self, usd_file_name: str) -> str:
        """
        로봇 USD 파일의 이름을 받아서, 운용 환경에 따른 USD 파일의 full path를 반환
        """
        return os.path.join(self.resources_directory, 'assets', 'robot', usd_file_name)

    def get_competition_logging_path(self) -> str:
        """
        로봇 USD 파일의 이름을 받아서, 운용 환경에 따른 USD 파일의 full path를 반환
        """
        return os.path.join(self.runtime_data_directory, 'competition_log')

    def get_runtime_data_directory_path(self) -> str:
        """
        운용 환경에 따른 런타임 데이터 디렉토리의 full path를 반환
        """
        return self.runtime_data_directory  
    
    def get_resources_directory_path(self) -> str:
        """
        운용 환경에 따른 자산 디렉토리의 full path를 반환
        """
        return self.resources_directory


    def get_competition_hash_seed(self) -> str:
        """
        경진대회 해시 시드를 반환합니다.
        """
        return "metasejong2025-competitor-token-generator"  
    
    def get_competitor_interface_namespace(self) -> str:
        """
        경진대회 인터페이스 네임스페이스를 반환합니다.
        """
        return "metasejong2025"
    
    def get_is_docker(self) -> str:
        """
        도커 환경 여부를 반환합니다.
        """
        return self.is_docker

    def get_team_name(self) -> str:
        """
        팀 이름을 반환합니다.
        """
        return self._config_data.get("team_name", "No name")


    def get_scenario_id(self) -> str:
        """
        시나리오의 고유 ID를 반환합니다.
        """
        return self.scenario_id

    def get_scenario_info(self) -> dict:
        """
        시나리오의 기본 정보를 반환합니다.
        
        Returns:
            dict: 다음 정보를 포함하는 딕셔너리
                - name (str): 시나리오 이름
                - description (str): 시나리오 설명
                - time_constraint (int): 시간 제약 (분)
        """
        scenario = self._scenario_data.get("scenario", {})

        # 필수값인 시간 제약이 없는 경우 예외 처리
        check_missing_mandatory_key(scenario, ["time_constraint"])
        
        return {
            "name": scenario.get("name", "No name"),
            "description": scenario.get("description", "No description"),
            "time_constraint": scenario.get("time_constraint")
        }
    
    def get_playground(self) -> dict:
        """
        시나리오의 플레이 그라운드 정보를 반환합니다.
        """
        playground = self._scenario_data.get("playground", {})

        # 필수값인 usd file 경로가 없는 경우 예외 처리
        check_missing_mandatory_key(playground, ["usd_file", "view_point"])

        # 필수값인 카메라 view angle 값이 없는 경우 예외 처리
        view_point = playground.get("view_point", {})
        check_missing_mandatory_key(view_point, ["camera_view_eye", "camera_view_target"])

        return playground

    def get_view_point(self) :
        """
        카메라 시점 정보를 반환합니다.
        """
        view_point = self.get_playground().get("view_point", {})
        
        camera_view_eye = view_point.get("camera_view_eye", [])
        camera_view_target = view_point.get("camera_view_target", [])

        return camera_view_eye, camera_view_target

    def get_mission(self) -> list:
        """
            미션 데이터를 배열 형태로 반환합니다.
            미션 데이터는 다음과 같은 형태로 반환됩니다.
            Returns:
            [
                {
                    "area_name": "mission_area_name",
                    "camera_info": {
                        "focal_length": 30.0,
                        "horizontal_aperture": 40.0,
                        "ground_height": 1.0,
                        "position": [position_x, position_y, position_z],   
                        "rotation": [rotation_x, rotation_y, rotation_z],
                    },
                    "mission_objects": [
                        {
                            "object_name": "object_name",
                            "count": count
                        }
                    ]
                }
            ]
        """

        mission = self._scenario_data.get("mission", {})

        # 반환할 list 초기화
        mission_list = []

        # 시나리오 정의에서 fixed camera에 대한 일부 정보(resolution, focal_length)가 없는 경우, config에서 default로 정의된 값을 사용하기 위해 
        default_fixed_camera_info = self.get_fixed_camera_defaults()
        
        # 전체 mission 정의에 대하여, mission_id는 chungmu1과 같은 카메라 영역 이름, value에는 camera_info, mission_objects 정보가 담긴 딕셔너리
        for mission_id, mission_data in mission.items():

            # 필수 키 검사  
            check_missing_mandatory_key(mission_data, ["camera_info", "mission_objects"])

            # 필수 키 검사 통과한 경우, 각 키에 대한 정보 추출
            camera_info = mission_data.get("camera_info", {})
            mission_objects = mission_data.get("mission_objects", {})

            # camera_info에 대해 필수 하위속성 키 검사 
            check_missing_mandatory_key(camera_info, ["position", "rotation"])

            # camera_info에 focal_length, resolution 정의가 없다면, config에서 default로 정의된 값을 사용
            camera_info["focal_length"] = camera_info.get("focal_length", default_fixed_camera_info["default_focal_length"])
            camera_info["horizontal_aperture"] = camera_info.get("horizontal_aperture", default_fixed_camera_info["default_horizontal_aperture"])
            camera_info["ground_height"] = camera_info.get("ground_height", default_fixed_camera_info["default_ground_height"])

            ## default 값 적용 후, 필수 키 검사 
            check_missing_mandatory_key(camera_info, ["focal_length", "horizontal_aperture", "ground_height", "position", "rotation"])

            # mission_objects는 dict 타입을 list 타입으로 변환
            mission_object_list = [{"object_name": key, "count": value} for key, value in mission_objects.items()]
            mission_object_types = [item["object_name"] for item in mission_object_list]

            # 시나리오의 mission_objects 정의에 플랫폼에서 지원하지 않는 유형을 정의한 경우 오류
            self._check_use_of_unsupport_mission_object(mission_object_types)

            # 결과를 받는 쪽에서 loop 돌기 편하게 mission 정의를 list 형태로 반환 
            mission_list.append({
                "area_name": mission_id,    
                "camera_info": camera_info,
                "mission_objects": mission_object_list
            })
        
        return mission_list
    
    def get_mission_camera_configs(self) -> list:
        """
        미션 카메라 설정 정보를 반환합니다.
        """
        missions = self.get_mission()

        camera_configs = []
        
        # missions 딕셔너리의 각 미션에 대해
        for mission in missions:
            mission_area_name = mission["area_name"]
            camera_info = mission["camera_info"]

            camera_config = {
                "name": mission_area_name,
                "focal_length":camera_info['focal_length'],
                "horizontal_aperture":camera_info['horizontal_aperture'],
                "position": camera_info["position"],
                "rotation": camera_info["rotation"],
            }
            camera_configs.append(camera_config)
    
        return camera_configs

    def get_used_mission_object_types(self) -> list:
        """
        시나리오 정의에서 사용된 모든 미션 오브젝트 유형의 합집합을 list 형태로 반환합니다.
        """
        missions = self.get_mission()

        used_mission_object_type_set = set()

        for mission in missions:
            for mission_objects in mission["mission_objects"]:
                used_mission_object_type_set.add(mission_objects["object_name"])

        return list(used_mission_object_type_set)

    def get_robot_start_point(self) -> list:
        """
        로봇의 시작 위치 정보를 list 형태로 반환합니다.
        로봇의 시작 위치 정보는 다음과 같은 형태로 반환됩니다.
        Returns:
                [x, y, z] 좌표
        """
        robot = self._scenario_data.get("robot", {})

        # 필수 키 검사
        check_missing_mandatory_key(robot, ["start_point"])

        # 필수 키 검사 통과한 경우, 각 키에 대한 정보 추출
        start_point = robot.get("start_point", {})
        
        return start_point

    def get_fixed_camera_defaults(self) -> dict:
        """
        고정 카메라의 기본 설정 정보를 반환합니다.
        """
        fixed_camera = self._config_data.get("fixed_camera", {})

        # 필수 키 검사
        check_missing_mandatory_key(fixed_camera, ["default_camera_resolution", "default_focal_length"])

        return fixed_camera

    def get_mission_object_types(self) -> list:
        """
        시나리오에서 사용되는 미션 오브젝트 유형을 반환합니다.
        """
        mission_object_types = self._config_data.get("mission_object_types", [])

        if len(mission_object_types) == 0:
            raise ValueError("mission_object_types is empty")
        
        return self._config_data.get("mission_object_types", [])

    def _check_use_of_unsupport_mission_object(self, mission_object_list: list):
        """
        미션 오브젝트 유형 검사
        """
        # config.yaml에 정의된 지원하는 mission object type을 조회 
        supported_mission_object_types = self.get_mission_object_types()

        unsupported_mission_object = [item for item in mission_object_list if item not in supported_mission_object_types]

        if len(unsupported_mission_object) > 0: 
            raise ValueError(f"unsupported mission object types are used: {unsupported_mission_object}")

    def get_playground_usd_path(self) -> str:
        """
        플레이 그라운드 USD 파일 경로를 반환합니다.
        """
        playground = self.get_playground()
        usd_file = playground.get("usd_file", "")

        return os.path.join(self.resources_directory, 'models', usd_file)

    def get_robot_usd(self) -> dict:
        """
        로봇 자산 정보를 반환합니다.
        """
        robot_asset = self._config_data.get("robot_asset", {})

        # 필수 키 검사
        check_missing_mandatory_key(robot_asset, ["type", "usd_file", "scale"])

        return robot_asset

    def get_mission_object_assets(self) -> dict:
        """
        config.yaml에 정의된 임무 객체(재활용 쓰레기)에 대한 USD 자산 정보를 반환합니다.
        """
        mission_object_assets = self._config_data.get("mission_object_assets", {})

        for mission_object_type, mission_object_asset in mission_object_assets.items():
            # 필수 키 검사
            check_missing_mandatory_key(mission_object_asset, ["type", "usd_file", "scale"])

        return mission_object_assets

    def get_used_mission_object_usds(self) -> dict:
        """
        임무 객체(재활용 쓰레기)에 대한 USD 자산 정보를 반환합니다.
        """
        used_mission_object_types = self.get_used_mission_object_types()
        mission_object_assets = self.get_mission_object_assets()

        used_mission_object_assets = {key: mission_object_assets[key] for key in used_mission_object_types if key in mission_object_assets}

        return used_mission_object_assets
           

    def __str__(self) -> str:
        """Configurations 객체의 문자열 표현을 반환합니다."""

        supported_mission_object_types = self.get_mission_object_types()
        scenario_info = self.get_scenario_info()
        playground = self.get_playground()
        mission_count = len(self.get_mission())
        robot_start_point = self.get_robot_start_point()

        return (
            f"Configurations(\n"
            f"  Config(\n"
            f"    Config File Path: '{self.config_file_path}',\n"
            f"    Mission object list: {supported_mission_object_types}\n"
            f"  )\n"
            f"  Scenario(\n"
            f"    Scenario File Path: '{self.scenario_file_path}',\n"
            f"    Scenario Name: '{scenario_info['name']}',\n"
            f"    Description: '{scenario_info['description']}',\n"
            f"    Time Constraint: {scenario_info['time_constraint']} min,\n"
            f"    Playground: {playground}\n"
            f"    Number of Missions: {mission_count},\n"
            f"      Robot Start Point: {robot_start_point}\n"
            f"  )\n"
            f")"
        )
