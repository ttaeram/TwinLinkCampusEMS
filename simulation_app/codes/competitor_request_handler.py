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
import queue
import time
import json
from datetime import datetime
import numpy as np
from configurations import Configurations
from competitor_request_message import (
    CompetitorAppStartedResponsePayload, 
    CompetitorNotificationMessage, 
    CompetitorRequestErrorPayload, 
    CompetitorRequestMessage, 
    CompetitorResponseMessage, 
    MessageStatus, 
    MessageType, 
    ReportStage1CompletedResponsePayload,
    TimeConstraintExpiredPayload
)
from common_utils import (
    check_missing_mandatory_key,
    generate_md5_hash,
    validate_md5_hash
)

configurations = Configurations()

CONST_COMPETITOR_INTERFACE_NAMESPACE = configurations.get_competitor_interface_namespace()
MD5_SEED = configurations.get_competition_hash_seed()


CONST_TOPIC_COMPETITOR_REQUEST = f"/{CONST_COMPETITOR_INTERFACE_NAMESPACE}/competitor_request"
CONST_TOPIC_COMPETITOR_RESPONSE = f"/{CONST_COMPETITOR_INTERFACE_NAMESPACE}/competitor_response"
CONST_TOPIC_COMPETITOR_NOTIFICATION = f"/{CONST_COMPETITOR_INTERFACE_NAMESPACE}/competitor_notification"

# CONST_COMPETITOR_REQUEST_MESSAGE_TYPES의 index
CONST_REQUEST_MSG_ID_COMPETITOR_APP_STARTED = 0   

CONST_COMPETITOR_REQUEST_MESSAGE_TYPES = [
    'COMPETITOR_APP_STARTED',
    'READY_TO_END',
    'READY_TO_RESTART',
    'READY_TO_PAUSE',
    'READY_TO_RESUME',
    'READY_TO_STOP',
]

# CONST_COMPETITOR_NOTIFICATION_MESSAGE_TYPES index
CONST_NOTIFICATION_MSG_ID_TIME_CONSTRAINT_EXPIRED = 0   
CONST_NOTIFICATION_MSG_ID_COMPETITOR_REQUEST_ERROR = 1   

CONST_COMPETITOR_NOTIFICATION_MESSAGE_TYPES = [
    'TIME_CONSTRAINT_EXPIRED',
    'COMPETITOR_REQUEST_ERROR',
]


CONST_ACTION_RESPONSE_STATUS_SUCCESS = 'success'
CONST_ACTION_RESPONSE_STATUS_ERROR = 'error'


class CompetitorRequestHandler:

    def __init__(self, node: Node):
        self._node = node
        self.message_queue = queue.Queue()

        self._subscriber = self._node.create_subscription(
            String, 
            CONST_TOPIC_COMPETITOR_REQUEST, 
            self.competitor_request_message_handler, 
            10
        )
        self.spinning = threading.Event()

        self._publisher_response  = self._node.create_publisher(
            String, 
            CONST_TOPIC_COMPETITOR_RESPONSE, 
            10
        )

        self._publisher_notification  = self._node.create_publisher(
            String, 
            CONST_TOPIC_COMPETITOR_NOTIFICATION, 
            10
        )

        self.request_handler_functions = {}

        self.request_handler_functions[MessageType.COMPETITOR_APP_STARTED.value] = self._handle_competitor_app_started
        self.request_handler_functions[MessageType.REPORT_STAGE1_COMPLETED.value] = self._handle_report_stage1_completed

        self.configurations = Configurations()

        self.logger = None
        self.competitor_team = None
        self.competitor_token = None
        self.competitor_session = None
        self.competitor_apply_stage = None

        self.competition_start_time = None
        self.competition_stage1_score = 0.0
        self.competition_stage2_score = 0.0

        self.log_file = None
        self.start_time = None

        self.log_dir = self.configurations.get_competition_logging_path()




    def start_competition(self, team, token, stage):

        self.competitor_team = team
        self.competitor_token = token
        self.competitor_apply_stage = stage

        self.competition_start_time = datetime.now()
        
        time_constraint_min = self.configurations.get_scenario_info().get("time_constraint")
        self._start_time_constraint_timer(time_constraint_min)

    def get_team_info(self):

        team_name = self.competitor_team
        apply_stage = self.competitor_apply_stage

        return team_name, apply_stage
    
    def _start_time_constraint_timer(self, time_constraint_min):
        time_constraint_sec = time_constraint_min * 60

        self.time_constraint_timer = threading.Timer(time_constraint_sec, self._time_constraint_handler)
        self.time_constraint_timer.start()

    def _stop_time_constraint_timer(self):
        self.time_constraint_timer.cancel() 

    def _time_constraint_handler(self):
        notification_message = CompetitorNotificationMessage(
            msg=MessageType.TIME_CONSTRAINT_EXPIRED,
            session=self.competitor_session,
            payload=TimeConstraintExpiredPayload(
                dummy='dummy'
            )
        )
        self._send_notification_message(notification_message)
        self.message_queue.put(notification_message)


    def stop_competition(self):
        self._stop_time_constraint_timer()
        self.stop_ros2()


    def spin_thread(self):
        self.spinning.set()

        while self.spinning.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_node()
    
    def start_ros2(self):
        self.ros2_thread = threading.Thread(target=self.spin_thread)
        self.ros2_thread.start()

    def stop_ros2(self):
        self.spinning.clear()
        self.ros2_thread.join()

    def update(self):
        try : 
            while not self.message_queue.empty():
                data = self.message_queue.get_nowait()
        except queue.Empty: 
            pass
        except Exception as e:
            traceback.print_exc()
            raise e

    def update_one(self):
        try : 
            data = self.message_queue.get_nowait()
            return data
        except queue.Empty: 
            pass
        except Exception as e:
            traceback.print_exc()
            raise e

    def start_main_task(self):
        while self.spinning.is_set():
            self.spin_thread()

    def set_answer_sheet(self, answer_sheet):
        self.answer_sheet = answer_sheet

    # 경진대회 참가자 앱에서 보내는 요청 메시지 처리 
    def competitor_request_message_handler(self, string_msg):
        request_message = None
        try:
            request_message_json = string_msg.data
            request_message = CompetitorRequestMessage.from_json(request_message_json)

            # 요청 메시지 타입이 유효한지 검사
            if request_message.msg not in MessageType:
                error_message = f"Invalid message type: {request_message.msg}"
                raise ValueError(error_message) 

            # 요청 메시지 세션 검증 
            self._validate_request_session(request_message)

            # 요청 메시지 타입에 따른 처리 함수 검증    
            handler_function = self.request_handler_functions[request_message.msg.value]

            # 요청 메시지 타입에 따른 처리 함수가 구현되어 있는지 검사  
            if handler_function is None:
                # 요청 메시지 타입에 따른 처리 함수가 구현되어 있지 않으면 에러 발생
                error_message = f"Not implemented message type: {request_message.msg.name}"
                raise ValueError(error_message) 


            # 요청 메시지 처리후 response 메시지 획득  
            response_message = handler_function(request_message)

            self.message_queue.put(request_message)

            self._send_response_message(response_message)
            
        except Exception as e:
            traceback.print_exc()

            msg = MessageType.COMPETITOR_REQUEST_ERROR
            if request_message is not None:
                if request_message.msg.value == MessageType.COMPETITOR_APP_STARTED.value:
                    msg = MessageType.COMPETITOR_APP_STARTED_RESPONSE
                elif request_message.msg.value == MessageType.REPORT_STAGE1_COMPLETED.value:
                    msg = MessageType.REPORT_STAGE1_COMPLETED_RESPONSE
                elif request_message.msg.value == MessageType.REPORT_STAGE2_COMPLETED.value :
                    msg = MessageType.REPORT_STAGE2_COMPLETED_RESPONSE

            if msg == MessageType.COMPETITOR_REQUEST_ERROR:
                notification_message = CompetitorNotificationMessage(
                    msg=msg,
                    session=request_message.session,
                    payload=CompetitorRequestErrorPayload(
                        error_message=str(e)
                    )
                )
                self._send_notification_message(notification_message)

            else:
                response_message = CompetitorResponseMessage(
                    msg=msg,
                    status=MessageStatus.FAILED,
                    status_message=str(e),
                    result={

                    }
                )
           
                self._send_response_message(response_message)

    def _send_response_message(self, response_message: CompetitorResponseMessage):
        response = String()
        response.data = response_message.to_json()

        self._publisher_response.publish(response)

    def _send_notification_message(self, notification_message: CompetitorNotificationMessage):
        message = String()
        message.data = notification_message.to_json()

        self._publisher_notification.publish(message)

    # 토큰 검증
    def _validate_request_token(self, team, token):
        expected_token = generate_md5_hash(team, CONST_COMPETITOR_INTERFACE_NAMESPACE)
        if token != expected_token:
            raise ValueError(f"Invalid token for '{team}' team: {token}")

    # 세션 생성
    def _generate_request_session(self, team):
        token = generate_md5_hash(team, CONST_COMPETITOR_INTERFACE_NAMESPACE)
        session_key = generate_md5_hash(team, token)

        session = team + ':' + session_key
        return session

    # 세션 검증 
    def _validate_request_session(self, request_message: CompetitorRequestMessage):

        # COMPETITOR_APP_STARTED 메시지의 경우 세션 검증 없이 통과  
        if request_message.msg.value == MessageType.COMPETITOR_APP_STARTED.value:
            return 
        
        # 세션 포맷 검증
        session_tokens = request_message.session.split(':')
        if len(session_tokens) != 2:
            raise ValueError(f"Invalid session format: {request_message.session}")
        
        # session에서 team과 token을 분리
        team = session_tokens[0]
        session_key = session_tokens[1]

        expected_session_key = self._generate_request_session(team)

        return session_key == expected_session_key


    def _handle_report_stage1_completed(self, request_message:CompetitorRequestMessage) -> CompetitorResponseMessage:

        # object_detections은 {class_name: "can", position: [x, y, z]} 형식의 리스트    
        object_detections = request_message.payload.object_detections

        # 객체인식 성공으로 인정되는 ground truth 위치와 객체 간의 거리 threshold 값 
        #   0.5 = 50cm 
        CONST_OBJECT_DETECTION_DISTANCE_THRESHOLD = 0.5

        deployed_mission_objects = self.answer_sheet.get("mission_objects")

        total_deployed_recyclable_mission_objects = 0
        # 배치된 미션 객체 중 하나씩 반복 처리
        for mission_object in deployed_mission_objects:
            recyclable = mission_object.get("recyclable")
            if not recyclable:
                continue

            total_deployed_recyclable_mission_objects += 1


        self.competition_stage1_score = 0
        total_deployed_recyclable_mission_objects = 0

        # 배치된 미션 객체 중 하나씩 반복 처리
        for mission_object in deployed_mission_objects:
            recyclable = mission_object.get("recyclable")
            if not recyclable:
                continue

            total_deployed_recyclable_mission_objects += 10

            deployed_class_name = mission_object.get("class_name")
            deployed_position = mission_object.get("position")

            # 객체인식 결과 중 하나씩 반복 처리
            for object_detection in object_detections:
                detected_class_name = object_detection.get("class_name")
                detected_position = object_detection.get("position")

                # 배치된 미션 객체와 객체인식 결과 간의 거리 계산
                distance = np.linalg.norm(np.array(detected_position) - np.array(deployed_position))

                # 거리가 threshold 값 이하이면 객체인식 성공으로 인정
                if distance < CONST_OBJECT_DETECTION_DISTANCE_THRESHOLD:
                    # 배치된 미션 객체와 객체인식 결과 간의 클래스 이름이 같으면 객체인식 성공으로 인정
                    if deployed_class_name == detected_class_name:
                        self.competition_stage1_score += 10
                        break

        self.competition_stage1_score = (self.competition_stage1_score / total_deployed_recyclable_mission_objects) * 100

        result = CompetitorResponseMessage(
            msg=MessageType.REPORT_STAGE1_COMPLETED_RESPONSE,
            status=MessageStatus.SUCCESS,
            status_message="OK",
            result=ReportStage1CompletedResponsePayload(dummy=f"{self.competition_stage1_score}")
        )

        return result
    
    def get_time_elapsed(self):
        if self.competition_start_time is None:
            return None
        
        return datetime.now() - self.competition_start_time

    def get_stage_scores(self):
        return self.competition_stage1_score, self.competition_stage2_score

    def get_recycled_status(self):

        deployed_mission_objects = self.answer_sheet.get("mission_objects")

        total_deployed_recyclable_mission_objects = 0
        # 배치된 미션 객체 중 하나씩 반복 처리
        for mission_object in deployed_mission_objects:
            recyclable = mission_object.get("recyclable")
            if not recyclable:
                continue

            total_deployed_recyclable_mission_objects += 1

        return 0, total_deployed_recyclable_mission_objects

    # COMPETITOR_APP_STARTED 메시지 처리 
    def _handle_competitor_app_started(self, request_message:CompetitorRequestMessage) -> CompetitorResponseMessage:

        payload = request_message.payload
        # 토큰 검증: 별도의 검증용 DB를 관리하지 않게 하기 위해서 team 이름으로부터 drive될 수 있는 문자열로 함 
        self._validate_request_token(payload.team, payload.token)

        # session id 발행: 역시 session id도 별도의 db를 둘건 아니라서 team 이름으로부터 drive될 수 있는 문자열로 함 
        session_key = generate_md5_hash(payload.team, payload.team+CONST_COMPETITOR_INTERFACE_NAMESPACE)
        session = payload.team + ':' + session_key
        self.competitor_session = session

        # 실제 시뮬레이션 앱에 요청할 내용이 있다면 호출(예: robot 제어, arm 제어, ... )
        # COMPETITOR_APP_STARTED 메시지의 경우 시뮬레이션 앱에서 CCTV 카메라 영상 출력 시작 요청 등 

        response_message = CompetitorResponseMessage(
            msg=MessageType.COMPETITOR_APP_STARTED_RESPONSE,
            status=MessageStatus.SUCCESS,
            status_message="OK",
            result=CompetitorAppStartedResponsePayload(session=session)
        )

        return response_message

    # JSON 문자열을 딕셔너리로 파싱
    def _parse_json(self, json_string):
        """JSON 문자열을 딕셔너리로 파싱합니다."""
        try:
            return json.loads(json_string)
        except json.JSONDecodeError as e:
            traceback.print_exc()
            return {}

    # 딕셔너리를 JSON 문자열로 직렬화
    def _serialize_to_json(self, data_dict):
        """딕셔너리를 JSON 문자열로 직렬화합니다."""
        try:
            return json.dumps(data_dict)
        except (TypeError, ValueError) as e:
            traceback.print_exc()
            return ""
