# emqx_pub.py

import random
import json
import os
import time
from pathlib import Path
from dotenv import load_dotenv
from paho.mqtt import client as mqtt_client

# Load environment variables from .env in same directory
env_path = Path(__file__).resolve().parent / ".env"
load_dotenv(dotenv_path=env_path)

# Load config
broker = os.getenv('BROKER')
username = os.getenv('USERID')
password = os.getenv('PASSWORD')
port = int(os.getenv('PORT', 8883))
topic = os.getenv('TOPIC', 'python/mqtt')  # default topic

class EmqxPublisher:
    def __init__(self, broker=broker, port=port, username=username, password=password, topic=topic):
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        self.client = self._connect_mqtt()

    def _connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("[MQTT] Connected to broker")
            else:
                print(f"[MQTT] Failed to connect, return code {rc}")

        client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client

    def start(self):
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()

    def publish(self, message: dict, retain: bool = True):
        msg_json = json.dumps(message)
        result = self.client.publish(self.topic, msg_json, retain=retain)
        status = result[0]
        if status == 0:
            print(f"[MQTT] Sent to `{self.topic}` → {msg_json}")
        else:
            print(f"[MQTT] Failed to send message to `{self.topic}`")

# Optional test run
def main():
    publisher = EmqxPublisher()
    publisher.start()

    message = {
        "robot_id": "turtlebot4",
        "status": "arrived",
        "timestamp": int(time.time())
    }
    publisher.publish(message)
    time.sleep(2)
    publisher.stop()

if __name__ == '__main__':
    main()
