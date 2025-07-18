# import random
# import json
# from paho.mqtt import client as mqtt_client
# from dotenv import load_dotenv
# import os

# load_dotenv()

# broker = os.getenv('BROKER')
# username = os.getenv('USERID')
# password = os.getenv('PASSWORD')
# port =  int(os.getenv('PORT'))

# topic = "python/mqtt"
# client_id = f'python-mqtt-{random.randint(0, 100)}'

# def connect_mqtt() -> mqtt_client.Client:
#     def on_connect(client, userdata, flags, rc):
#         if rc == 0:
#             print("Connected to MQTT Broker!")
#             client.subscribe(topic)
#         else:
#             print("Failed to connect, return code %d\n", rc)

#     client = mqtt_client.Client(
#         client_id=client_id,
#         protocol=mqtt_client.MQTTv311
#     )
#     client.tls_set()
#     client.username_pw_set(username, password)
#     client.on_connect = on_connect
#     return client

# def run_test():
#     client = connect_mqtt()

#     def on_message(client, userdata, msg):
#         payload_str = msg.payload.decode()
#         try:
#             data = json.loads(payload_str)
#             # print(f"Received JSON: {payload_str}")
#             print(f"id = {data['id']}, status = {data['status']}")
#         except json.JSONDecodeError:
#             print(f"Received non-JSON message: {payload_str}")

#     client.on_message = on_message
#     client.connect(broker, port)
#     client.loop_forever()

# def run(callback_function):
#     client = connect_mqtt()

#     # def on_message(client, userdata, msg):
#     #     payload_str = msg.payload.decode()
#     #     try:
#     #         data = json.loads(payload_str)
#     #         # print(f"Received JSON: {payload_str}")
#     #         print(f"id = {data['id']}, status = {data['status']}")
#     #     except json.JSONDecodeError:
#     #         print(f"Received non-JSON message: {payload_str}")

#     client.on_message = callback_function
#     client.connect(broker, port)
#     client.loop_forever()

# if __name__ == '__main__':
#     run_test()

import random
import json
from paho.mqtt import client as mqtt_client
from dotenv import load_dotenv
import os

load_dotenv()

broker = os.getenv('BROKER')
username = os.getenv('USERID')
password = os.getenv('PASSWORD')
port =  int(os.getenv('PORT'))

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

def runTest():
    client = connect_mqtt()

    def on_message(client, userdata, msg):
        payload_str = msg.payload.decode()
        try:
            data = json.loads(payload_str)
            print(f"[RAW MESSAGE] {data}")  # 메시지 내용 확인
            print(f"id = {data['id']}, status = {data['status']}")
        except json.JSONDecodeError:
            print(f"❌ Invalid JSON: {payload_str}")
        except KeyError as e:
            print(f"⚠️ KeyError: '{e}' not found in message: {data}")

    client.on_message = on_message
    client.connect(broker, port)
    client.loop_forever()

def run(callback_function):
    client = connect_mqtt()

    # def on_message(client, userdata, msg):
    #     payload_str = msg.payload.decode()
    #     try:
    #         data = json.loads(payload_str)
    #         print(f"[RAW MESSAGE] {data}")  # 메시지 내용 확인
    #         print(f"id = {data['id']}, status = {data['status']}")
    #     except json.JSONDecodeError:
    #         print(f"❌ Invalid JSON: {payload_str}")
    #     except KeyError as e:
    #         print(f"⚠️ KeyError: '{e}' not found in message: {data}")

    client.on_message = callback_function
    client.connect(broker, port)
    client.loop_forever()

if __name__ == '__main__':
    runTest()
