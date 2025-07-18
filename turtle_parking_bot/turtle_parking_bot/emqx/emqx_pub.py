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
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
    client.tls_set()
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client , message):
    msg_json = json.dumps(message)
    result = client.publish(topic, msg_json, retain=True)
    status = result[0]
    if status == 0:
        print(f"Sent `{msg_json}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    import time
    time.sleep(2)
    client.loop_stop()

if __name__ == '__main__':
    run()
