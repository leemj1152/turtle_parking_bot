# turtle_parking_bot/emqx/subscriber.py

import random
import json
import os
from pathlib import Path
from dotenv import load_dotenv
from paho.mqtt import client as mqtt_client

class EmqxSubscriber:
    def __init__(self, callback=None):
        self._load_env()

        self.broker = os.getenv('BROKER')
        self.username = os.getenv('USERID')
        self.password = os.getenv('PASSWORD')
        self.port = int(os.getenv('PORT', 8883))
        self.topic = os.getenv('TOPIC', 'python/mqtt')
        self.client_id = f'python-mqtt-sub-{random.randint(0, 100)}'

        self.callback = callback or self.default_callback
        self.client = self._connect_mqtt()

    def _load_env(self):
        base_path = Path(__file__).resolve().parent
        env_path = base_path / ".env"

        if not env_path.exists():
            first_prefix = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
            env_path = Path(first_prefix) / 'share' / 'turtle_parking_bot' / 'emqx' / '.env'

        if env_path.exists():
            load_dotenv(dotenv_path=env_path)
        else:
            print(f"⚠️ .env file not found at {env_path}")

    def _connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("✅ Connected to MQTT Broker")
                client.subscribe(self.topic)
            else:
                print(f"❌ Failed to connect, return code {rc}")

        client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.on_message = self.callback
        return client

    def default_callback(self, client, userdata, msg):
        payload_str = msg.payload.decode()
        try:
            data = json.loads(payload_str)
            print(f"[MQTT] id = {data.get('id', 'N/A')}, status = {data.get('status', 'N/A')}")
        except json.JSONDecodeError:
            print(f"[MQTT] Non-JSON message received: {payload_str}")

    def run(self):
        self.client.connect(self.broker, self.port)
        self.client.loop_forever()
