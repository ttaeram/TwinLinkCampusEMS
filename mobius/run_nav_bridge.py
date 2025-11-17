import json

# Import your original subscriber module
import mqtt_subscribe  # must be on PYTHONPATH or same dir

# Import the bridge handler
from nav_bridge_handler import init_bridge, process_mobius_payload


# Keep original on_message ref
_original_on_message = mqtt_subscribe.on_message

def _wrapped_on_message(client, userdata, msg):
    # 1) 기존 on_message 먼저 호출(그대로 유지)
    try:
        _original_on_message(client, userdata, msg)
    except Exception as e:
        print("original on_message error:", e)

    # 2) 브릿지로 넘기기 전에 'pc' 래퍼 제거
    try:
        payload_str = msg.payload.decode("utf-8")
        payload = json.loads(payload_str)

        if isinstance(payload, dict) and "pc" in payload and isinstance(payload["pc"], dict):
            payload = payload["pc"]

        process_mobius_payload(payload)
    except Exception as e:
        print("bridge on_message error:", e)


def main():
    # Start ROS2 bridge in background
    init_bridge()

    # Monkey-patch the on_message before starting subscribing()
    mqtt_subscribe.on_message = _wrapped_on_message

    # Run subscribing exactly as the original file would
    # topic default in original __main__ was 'sub_data'
    mqtt_subscribe.subscribing('sub_data')


if __name__ == "__main__":
    main()
