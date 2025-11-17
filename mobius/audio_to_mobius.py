import os
import time
import base64
import configparser
from datetime import datetime, timezone

from mqtt_publish import publish_cin_only


# ========= 환경 설정 =========

WATCH_DIR = os.environ.get("AUDIO_SAVE_DIR", "/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/audios/command")
os.makedirs(WATCH_DIR, exist_ok=True)

# config.ini 경로
INI_PATH = '/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini'
config = configparser.ConfigParser()
if not config.read(INI_PATH):
    raise FileNotFoundError(f"config.ini를 찾을 수 없습니다: {INI_PATH}")

IOTPLATFORM_IP = config['API']['IOTPLATFORM_IP']
IOTPLATFORM_HTTP_PORT = int(config['API']['IOTPLATFORM_MQTT_PORT'])

# oneM2M 경로 설정
CSE_BASE = '/Mobius'
AE_NAME = 'CampusEMS'
CNT_CHAIN = ['ControlCenter', 'Manager', 'VoiceRawData']
AE_ID = 'CAdmin'
CIN_URI = f"{CSE_BASE}/{AE_NAME}/" + "/".join(CNT_CHAIN)


# ========= 유틸 함수 =========

def encode_audio_file(filepath: str) -> str:
    with open(filepath, "rb") as f:
        raw = f.read()
    b64 = base64.b64encode(raw).decode("ascii")
    return b64


def build_payload_from_file(filepath: str) -> dict:
    filename = os.path.basename(filepath)
    stat = os.stat(filepath)

    b64_data = encode_audio_file(filepath)

    payload = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "filename": filename,
        "size_bytes": stat.st_size,
        "mime_type": "audio/wav",
        "encoding": "base64",
        "data": b64_data,
    }
    return payload


def send_file_to_mobius(filepath: str):
    payload = build_payload_from_file(filepath)
    try:
        publish_cin_only(CIN_URI, AE_ID, payload)
        print(f"[Mobius] CIN created -> {CIN_URI}")
        print(f"          filename = {payload['filename']}, size={payload['size_bytes']} bytes")
        print(f"          base64 length = {len(payload['data'])}")
    except Exception as e:
        print(f"[Mobius] CIN 전송 실패: {e}")


def watch_directory(interval: float = 1.0):
    print(f"[Watcher] watching directory: {WATCH_DIR}")
    processed = set()

    # 시작 시점에 이미 있던 파일은 processed에 넣고 시작
    for name in os.listdir(WATCH_DIR):
        if name.lower().endswith(".wav"):
            processed.add(name)

    while True:
        try:
            current = set(
                name for name in os.listdir(WATCH_DIR)
                if name.lower().endswith(".wav")
            )

            new_files = current - processed
            for name in sorted(list(new_files)):
                filepath = os.path.join(WATCH_DIR, name)
                print(f"[Watcher] new file detected: {filepath}")
                send_file_to_mobius(filepath)
                processed.add(name)

            time.sleep(interval)
        except KeyboardInterrupt:
            print("\n[Watcher] stopped by user.")
            break
        except Exception as e:
            print(f"[Watcher] error: {e}")
            time.sleep(interval)


if __name__ == "__main__":
    watch_directory(interval=1.0)
