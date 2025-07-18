import random
import json
from paho.mqtt import client as mqtt_client

broker = 't1f11760.ala.us-east-1.emqxsl.com'
port = 8883
username = 'emqx'
password = 'rokey1234'

topic = "python/mqtt"
client_id = f'python-mqtt-{random.randint(0, 100)}'

def connect_mqtt() -> mqtt_client.Client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(
        client_id=client_id,
        protocol=mqtt_client.MQTTv311
    )
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
            # print(f"Received JSON: {payload_str}")
            print(f"id = {data['id']}, status = {data['status']}")
        except json.JSONDecodeError:
            print(f"Received non-JSON message: {payload_str}")

    client.on_message = on_message
    client.connect(broker, port)
    client.loop_forever()

def run(callback_function):
    client = connect_mqtt()

    # def on_message(client, userdata, msg):
    #     payload_str = msg.payload.decode()
    #     try:
    #         data = json.loads(payload_str)
    #         # print(f"Received JSON: {payload_str}")
    #         print(f"id = {data['id']}, status = {data['status']}")
    #     except json.JSONDecodeError:
    #         print(f"Received non-JSON message: {payload_str}")

    client.on_message = callback_function
    client.connect(broker, port)
    client.loop_forever()

if __name__ == '__main__':
    run()
