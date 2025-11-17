# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import logging
from datetime import datetime
import os
from typing import Optional
from configurations import Configurations
from common_utils import generate_md5_hash, validate_md5_hash

########################################################
# 경연에 대한 전체 프로세스를 관리하고, 진행 과정을 파일로 기록한다. 
########################################################    
class CompetitionManager:
    _instance = None
    _initialized = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(CompetitionManager, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        # 다양한 프로세스에서 공유하여 사용하기 위하여 싱글톤 패턴으로 구현
        # Singleton 패턴: 이미 초기화된 경우 skip
        if CompetitionManager._initialized:
            return
            
        self.configurations = Configurations()

        self.logger = None
        self.competitor_name = None
        self.log_file = None
        self.start_time = None

        self.log_dir = self.configurations.get_competition_logging_path()


        CompetitionManager._initialized = True
    

    def start_competition(self, competitor_name: str, token: str, apply_level: int) -> None:
        self._start_competition_logging(competitor_name)

        self.competitor_name = competitor_name
        self.competitor_token = token
        self.apply_level = apply_level


        expected_token = generate_md5_hash(competitor_name, self.configurations.get_competition_hash_seed())
        if token != expected_token:
            raise ValueError(f"Invalid token for '{competitor_name}' team: {token}")


        self.competitor_session = competitor_name + ":" + token
        self.start_time = datetime.now()
        self.score = 0

    def stop_competition(self) -> None:
        self._stop_competition_logging()



    # 경연 시작    
    def _start_competition_logging(self, competitor_name: str) -> None:
        """경연 시작
        Args:
            competitor_name (str): 참가자 이름
            log_dir (str, optional): 로그 저장 디렉토리. Defaults to 'competition_logs'.
        """
        self.competitor_name = competitor_name
        self.start_time = datetime.now()
        
        # 로그 디렉토리 생성
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # 로그 파일명: competitor_name_YYYYMMDD_HHMMSS.log
        timestamp = self.start_time.strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f"{competitor_name}_{timestamp}.log")
        
        # 로거 설정
        self.logger = logging.getLogger(f"competition_{competitor_name}")
        self.logger.setLevel(logging.INFO)
        
        # 파일 핸들러 설정
        file_handler = logging.FileHandler(self.log_file)
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
        file_handler.setFormatter(formatter)
        
        self.logger.addHandler(file_handler)
        
        # 경연 시작 로그
        self.info(f"Competition started for competitor: {competitor_name}")
    
    def _stop_competition_logging(self) -> None:
        """경연 로깅 종료"""
        if self.logger:
            end_time = datetime.now()
            duration = end_time - self.start_time
            self.info(f"Competition ended for competitor: {self.competitor_name}")
            self.info(f"Total duration: {duration}")
            
            # 핸들러 정리
            for handler in self.logger.handlers[:]:
                handler.close()
                self.logger.removeHandler(handler)
            
            self.logger = None
            self.competitor_name = None
            self.log_file = None
            self.start_time = None
    
    def _log(self, level: str, message: str) -> None:
        """내부 로깅 함수
        Args:
            level (str): 로그 레벨 (INFO, MSG, SCORE, ERROR)
            message (str): 로그 메시지
        """
        if self.logger:
            self.logger.info(f"[{level}] {message}")
        else:
            pass
    
    def info(self, message: str) -> None:
        """정보 로그
        Args:
            message (str): 로그 메시지
        """
        self._log("INFO", message)
    
    def msg(self, message: str) -> None:
        """메시지 로그
        Args:
            message (str): 로그 메시지
        """
        self._log("MSG", message)
    
    def score(self, message: str) -> None:
        """점수 로그
        Args:
            message (str): 로그 메시지
        """
        self._log("SCORE", message)
    
    def error(self, message: str) -> None:
        """에러 로그
        Args:
            message (str): 로그 메시지
        """
        self._log("ERROR", message)
