import paho.mqtt.client as mqtt
import json
import random
import configparser

# Config 파일 로드
config = configparser.ConfigParser()
config.read('/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini')

# API 섹션에서 기본 변수 로드
IOTPLATFORM_IP = config['API']['IOTPLATFORM_IP']
IOTPLATFORM_MQTT_PORT = config['API']['IOTPLATFORM_MQTT_PORT']
IOTPLATFORM_URL_MQTT = config['API']['IOTPLATFORM_URL_MQTT']

def _normalize_to_for_mqtt(uri: str) -> str:
    # MQTT 경로는 'Mobius/...' 형태로
    if isinstance(uri, str) and uri.startswith('/'):
        return uri[1:]
    return uri

# cnt(container) 생성
def crt_cnt(URI, AE_ID, resourceName):
    rand = str(int(random.random()*100000)) 
    
    crt_cnt = {
                "to": URI,
                "fr": AE_ID,
                "op":1,
                "ty":3,
                "rqi": rand,
                "pc":{
                    "m2m:cnt": {
                        "rn": resourceName
                        }
                    }
            }
    return crt_cnt


# sub(subscription) 생성
def crt_sub(URI, AE_ID, resourceName):
    rand = str(int(random.random()*100000))
    crt_sub = {
                    "to": URI,
                    "fr": AE_ID,
                    "op":1,
                    "ty":23,
                    "rqi": rand,
                    "pc":{
                        "m2m:sub": {
                            "rn": resourceName,
                            "enc":{"net":[3]},
                            "nu":[IOTPLATFORM_URL_MQTT.format(IOTPLATFORM_IP, resourceName)]
                            }
                        }
                }
    return crt_sub


# cin(content instance) 생성
def crt_cin(URI, AE_ID, data):
    if isinstance(data, (dict, list)):
        data = json.dumps(data, ensure_ascii=False)
    rand = str(int(__import__('random').random()*100000))
    return {
        "to": _normalize_to_for_mqtt(URI),
        "fr": AE_ID,
        "op": 1,
        "ty": 4,
        "rqi": rand,
        "pc": {"m2m:cin": {"con": data, "cnf": "application/json"}}
    }


# 기본 MQTT 콜백 함수
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)

def on_disconnect(client, userdata, flags, rc=0):
    print("Disconnected")

def on_publish(client, userdata, mid):
    print("In on_pub callback mid= ", mid)


def publishing(URI, AE_ID, resourceName = None, data = None):
    # 새로운 클라이언트 생성
    client = mqtt.Client()

    # 콜백 함수 설정 on_connect(브로커에 접속), on_disconnect(브로커에 접속중료), on_publish(메세지 발행)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish

    # ip_address : IOTPLATFORM_IP, port: IOTPLATFORM_MQTT_PORT 에 연결
    client.connect(IOTPLATFORM_IP, int(IOTPLATFORM_MQTT_PORT))

    # crt_cnt, crt_sub, crt_cin 함수를 통해 생성한 json 형식의 데이터를 string 형태로 변환
    create_cnt = json.dumps(crt_cnt(URI, AE_ID, resourceName))
    create_sub = json.dumps(crt_sub(URI, AE_ID, resourceName))
    create_cin = json.dumps(crt_cin(URI, AE_ID, data))

    # common topic 으로 메세지 발행
    client.publish('/oneM2M/req/' + AE_ID + '/Mobius2/json', create_cnt)
    client.publish('/oneM2M/req/' + AE_ID + '/Mobius2/json', create_sub)
    client.publish('/oneM2M/req/' + AE_ID + '/Mobius2/json', create_cin)

    # 연결 종료
    client.disconnect()

def publish_cin_only(URI, AE_ID, data):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish

    client.connect(IOTPLATFORM_IP, int(IOTPLATFORM_MQTT_PORT))

    # cin payload만 생성
    create_cin = json.dumps(crt_cin(URI, AE_ID, data), ensure_ascii=False)

    req_topic = f'/oneM2M/req/{AE_ID}/Mobius2/json'
    client.loop_start()
    client.publish(req_topic, create_cin)
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    publishing('Mobius/Meta-Sejong/Chungmu-hall/Sensor1/data', 'CAdmin', data = 'hi')
