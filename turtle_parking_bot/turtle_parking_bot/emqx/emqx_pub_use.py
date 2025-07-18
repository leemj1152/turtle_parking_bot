# my_task.py
from emqx_pub import EmqxPublisher
import time

def send_robot_status():
    pub = EmqxPublisher()
    pub.start()

    message = {
        "robot_id": "robot2",
        "status": "parking_complete",
        "timestamp": int(time.time())
    }
    pub.publish(message)

    pub.stop()

if __name__ == '__main__':
    send_robot_status()
