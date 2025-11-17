import logging
import threading
import pyaudio
from speechtotextmodule import speech2textmodule as s2t
from intentmakermodule_headset import IntentMaker
from speech_recognition import UnknownValueError
import time
from datetime import datetime
import os
import sys
import re

# mqtt_publish 재사용
from mqtt_publish import publish_cin_only

logging.getLogger("transformers.tokenization_utils_base").setLevel(logging.ERROR)

# ------------------------------
# 로봇 이동 명령 관련 설정
# ------------------------------

ROBOT_CODE_MAP = {
    "1001": "Robot01",
    "1002": "Robot07",
}

WAYPOINTS = {
    ("Robot01", 1): {
        "header": {
            "frame_id": "map"
        },
        "pose": {
            "position": {
                "x": 12.740621566772461,
                "y": 56.33754348754883,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.16512340204730916,
                "w": 0.9862729146115302
            }
        }
    },
    ("Robot07", 2): {
        "header": {
            "frame_id": "map"
        },
        "pose": {
            "position": {
                "x": 57.85000114962459,
                "y": 24.740000485032798,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }
    }
}
# oneM2M Originator
AE_ID = "CAdmin"

def parse_robot_command(sentence: str):
    m = re.search(r"(\d{4})\s*지점\s*(\d+)\s*로\s*이동", sentence)
    if not m:
        return None, None
    robot_code = m.group(1)
    point_id = int(m.group(2))
    return robot_code, point_id


class SuppressStderr:
    def __enter__(self):
        self.stderr_fd = sys.stderr.fileno()
        self.saved_stderr_fd = os.dup(self.stderr_fd)
        self.devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self.devnull_fd, self.stderr_fd)

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.dup2(self.saved_stderr_fd, self.stderr_fd)
        os.close(self.devnull_fd)
        os.close(self.saved_stderr_fd)


###### 2025.11.14 수정
def list_microphones():
    with SuppressStderr():
        p = pyaudio.PyAudio()
        info = p.get_host_api_info_by_index(0)
        num_devices = info.get('deviceCount')

        print("Available microphones:")
        for i in range(num_devices):
            device_info = p.get_device_info_by_host_api_device_index(0, i)
            if device_info.get('maxInputChannels') > 0:
                print(f"Index {i}: {device_info.get('name')}")

        p.terminate()


# 음성인식 시작하는 함수
def start_recognition(s2t, stop_event):
    audio = s2t.recognize_command(stop_event)
    return audio


##### 수정함(11.13)
def transcribe(s2t, audio, previous_uuid=None):
    try:
        start_time = time.time()
        sentence = s2t.speech2text(audio)
        stop_time = time.time()
        delta_time = stop_time - start_time
        print(f'시간측정: {delta_time}')

        if sentence:
            print(f"[STT] sentence: {sentence}")
            intent_maker = IntentMaker(sentence, previous_uuid, "KETI_GCS")

            try:
                device_list, command = intent_maker.get_device_list()
                intent, date_string, new_uuid = intent_maker.intent_maker(device_list, command)

            except ValueError as ve:
                print("IntentMaker에서 ValueError 발생: ", ve)
                device_list = []
                command = " "
                intent, date_string, new_uuid = intent_maker.intent_maker(device_list, command)

            except Exception as e:
                print("IntentMaker 오류: ", e)
                date_string = "%Y-%m-%dT%H:%M:%S,%f%z"
                new_uuid = None
                intent = None

            if "확인" not in sentence and "취소" not in sentence:
                intent_maker.set_previous_uuid(new_uuid)

            return sentence, date_string, new_uuid, intent

    except UnknownValueError:
        print("Speech Recognition could not understand audio")


# 음성인식을 항상 실행하는 클래스
class RecognitionHandler:
    def __init__(self, s2t, mic_index):
        with SuppressStderr():
            self.s2t = s2t(mic_index)

        self.s2t = s2t(mic_index)
        self.stop_event = threading.Event()
        self.recognition_thread = threading.Thread(target=self.recognition_worker)
        self.recognition_thread.start()
        self.results = None
        self.previous_uuid = None
        self.intent_uuid_list = [None, None]

        # 음성 파일 저장
        self.audio_save_dir = os.environ.get(
            "AUDIO_SAVE_DIR",
            "/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/audios/command"
        )
        os.makedirs(self.audio_save_dir, exist_ok=True)

    def save_audio(self, audio):
        if audio is None:
            return None

        if hasattr(audio, "get_wav_data"):
            wav_bytes = audio.get_wav_data()
        else:
            try:
                wav_bytes = bytes(audio)
            except Exception as e:
                print(f"[save_audio] audio를 bytes로 변환 실패: {e}")
                return None

        ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
        filename = f"{ts}.wav"
        filepath = os.path.join(self.audio_save_dir, filename)

        try:
            with open(filepath, "wb") as f:
                f.write(wav_bytes)
            print(f"[save_audio] saved: {filepath}")
            return filepath
        except Exception as e:
            print(f"[save_audio] 파일 저장 실패: {e}")
            return None

    def handle_robot_command(self, sentence: str):
        robot_code, point_id = parse_robot_command(sentence)
        if not robot_code or not point_id:
            return  # 이동 명령이 아님

        robot_name = ROBOT_CODE_MAP.get(robot_code)
        if not robot_name:
            print(f"[CMD] 알 수 없는 로봇 코드: {robot_code}")
            return

        pose_content = WAYPOINTS.get((robot_name, point_id))
        if not pose_content:
            print(f"[CMD] {robot_name}의 지점 {point_id} 좌표가 정의되어 있지 않습니다.")
            return

        # mqtt_publish.crt_cin() 안에서 con으로 들어갈 데이터
        data = {
            "pose": pose_content
        }

        # mqtt_publish.crt_cin() 에 들어갈 URI
        uri = f"/Mobius/CampusEMS/Robots/{robot_name}/Control/Nav"

        print(f"[CMD] '{sentence}' → {robot_name} 지점 {point_id} 이동 CIN publish")
        print(f"[CMD] URI={uri}, AE_ID={AE_ID}")
        try:
            publish_cin_only(uri, AE_ID, data)
        except Exception as e:
            print(f"[CMD] publish_cin_only 호출 실패: {e}")

    def recognition_worker(self):
        while not self.stop_event.is_set():
            with SuppressStderr():
                audio = start_recognition(self.s2t, self.stop_event)

            if audio:
                saved_path = self.save_audio(audio)

                self.results = transcribe(self.s2t, audio, self.previous_uuid)
                if self.results:
                    self.update_intent_list(self.results)

                    # STT 결과에서 이동 명령이 있는지 확인 후 MQTT로 Nav CIN 전송
                    try:
                        sentence, date_string, new_uuid, intent = self.results
                        self.handle_robot_command(sentence)
                    except Exception as e:
                        print(f"[RecognitionHandler] handle_robot_command 오류: {e}")

    def update_intent_list(self, results):
        if results:
            sentence, date_string, new_uuid, intent = results
            if "확인" in sentence or "취소" in sentence:
                self.intent_uuid_list = [None, None]
                self.previous_uuid = None 
            else:
                self.previous_uuid = new_uuid
                self.intent_uuid_list[1] = self.intent_uuid_list[0]
                self.intent_uuid_list[0] = {
                    "sentence": sentence,
                    "date_strinRecognitionHandlerg": date_string,
                    "new_uuid": new_uuid,
                    "intent": intent
                }

# main 함수
if __name__ == "__main__":
    list_microphones()
    mic_indices = [10]
    handlers = [RecognitionHandler(s2t, mic_index) for mic_index in mic_indices]

    for handler in handlers:
        handler.recognition_thread.join()
