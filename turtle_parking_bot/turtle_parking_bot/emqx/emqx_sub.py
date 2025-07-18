import random
import json
import os
from pathlib import Path
from dotenv import load_dotenv
from paho.mqtt import client as mqtt_client

# ✅ .env 파일 로드
load_dotenv(dotenv_path=Path(__file__).resolve().parent / ".env")

# ✅ 환경변수 로드
broker = os.getenv('BROKER')
username = os.getenv('USERID')
password = os.getenv('PASSWORD')
port = int(os.getenv('PORT', 8883))  # 기본값 8883 사용 가능

topic = "python/mqtt"
client_id = f'python-mqtt-{random.randint(0, 100)}'

def connect_mqtt() -> mqtt_client.Client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic)
        else:
            print(f"Failed to connect, return code {rc}")

    client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
    client.tls_set()
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    return client

def run():
    client = connect_mqtt()

    def on_message(client, userdata, msg):
        payload_str = msg.payload.decode()
        try:
            data = json.loads(payload_str)
            print(f"id = {data.get('id', 'N/A')}, status = {data.get('status', 'N/A')}")
        except json.JSONDecodeError:
            print(f"Received non-JSON message: {payload_str}")

    client.on_message = on_message
    client.connect(broker, port)
    client.loop_forever()

if __name__ == '__main__':
    run()
