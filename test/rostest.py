import json
import subprocess
import os
import hashlib

# CONST_COMPETITOR_REQUEST_MESSAGE_TYPES의 index
CONST_REQUEST_MSG_ID_COMPETITOR_APP_STARTED = 0   

CONST_COMPETITOR_REQUEST_MESSAGE_TYPES = [
    101,
    'READY_TO_END',
    'READY_TO_RESTART',
    'READY_TO_PAUSE',
    'READY_TO_RESUME',
    'READY_TO_STOP',
]


    
def generate_md5_hash(plaintext:str, seed:str):
    """plaintext 문자열을 seed 문자열을 키로 하여 md5 해시 값을 생성하는 함수"""
    return hashlib.md5(f"{plaintext}{seed}".encode()).hexdigest()

def validate_md5_hash(hash:str, plaintext:str, seed:str):
    """hash된 문자열과 plaintext + seed 문자열 조합을 비교하여 동일한지 검증하는 함수"""
    generated_hash = generate_md5_hash(plaintext, seed)
    return hash == generated_hash


CONST_COMPETITOR_REQUEST_MESSAGE_SAMPLES = [
    {
        "msg": CONST_COMPETITOR_REQUEST_MESSAGE_TYPES[CONST_REQUEST_MSG_ID_COMPETITOR_APP_STARTED],
        "session": "",
        "payload": {
            "team": "demo_team",
            "token": "18471a11421511d3c3a9f56c53bc8d57",
            "level": 2
        }
    }
]


def create_ros2_topic_command(topic_name, dict_data, once=False, rate=None):
    # 딕셔너리를 JSON 문자열로 변환
    json_str = json.dumps(dict_data, ensure_ascii=False)
    json_str = json_str.replace('"', '\\"')
    
    # 기본 명령어 구성
    command_parts = ['ros2', 'topic', 'pub']
    
    # 옵션 추가
    if once:
        command_parts.append('-1')
    if rate is not None:
        command_parts.extend(['--rate', str(rate)])
    
    # 토픽 이름과 메시지 타입, 데이터 추가
    command_parts.extend([
        topic_name,
        'std_msgs/msg/String',
        f'{{"data": "{json_str}"}}'
    ])
    print(f"Executing command: {' '.join(command_parts)}")

    try:
        # 방법 1: subprocess.run 사용 (권장)    
        result = subprocess.run(
            command_parts, 
            check=True, 
            capture_output=True, 
            text=True
        )
        print("Command output:", result.stdout)
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        print(f"Error output: {e.stderr}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False
    
# 사용 예시
if __name__ == "__main__":
    test_msg_id = CONST_REQUEST_MSG_ID_COMPETITOR_APP_STARTED
    test_dict = CONST_COMPETITOR_REQUEST_MESSAGE_SAMPLES[test_msg_id]
    
    # 다양한 옵션으로 명령어 생성
    topic_name = "/metasejong2025/competitor_request"
    
    
    # 한 번만 발행
    command2 = create_ros2_topic_command(topic_name, test_dict, once=True)
   

