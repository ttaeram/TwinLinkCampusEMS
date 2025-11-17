import paho.mqtt.client as mqtt
import json
import configparser

# Config 파일 로드
config = configparser.ConfigParser()
config.read('/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini')

# API 섹션에서 기본 변수 로드
IOTPLATFORM_IP = config['API']['IOTPLATFORM_IP']
IOTPLATFORM_MQTT_PORT = config['API']['IOTPLATFORM_MQTT_PORT']
IOTPLATFORM_URL_MQTT = config['API']['IOTPLATFORM_URL_MQTT']

# 기본 MQTT 콜백 함수
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)

def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))

def on_subscribe(client, userdata, mid, granted_qos):
    print("subscribed: " + str(mid) + " " + str(granted_qos))


# 서버에게서 PUBLISH 메시지를 받을 때 호출되는 콜백
def on_message(client, userdata, msg): 
    msg = msg.payload.decode("utf-8")
    msg = json.loads(msg)
    print(msg)

       
def subscribing(topic):
    # 새로운 클라이언트 생성
    client = mqtt.Client()
    
    # 콜백 함수 설정 on_connect(브로커에 접속), on_disconnect(브로커에 접속중료), on_subscribe(topic 구독), on_message(발행된 메세지가 들어왔을 때)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    
    # ip_address : IOTPLATFORM_IP, port: IOTPLATFORM_MQTT_PORT 에 연결
    client.connect(IOTPLATFORM_IP, int(IOTPLATFORM_MQTT_PORT))
    
    # common topic 으로 메세지 발행
    client.subscribe('/oneM2M/req/+/' + topic + '/#')
    client.loop_forever() # 네트웍 트래픽을 처리, 콜백 디스패치, 재접속 등을 수행하는 블러킹 함수
                          # 멀티스레드 인터페이스나 수동 인터페이스를 위한 다른 loop*() 함수도 있음


if __name__=="__main__":
    subscribing('sub_data')
