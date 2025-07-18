# my_task.py
# from emqx_pub import EmqxPublisher
from turtle_parking_bot.emqx.emqx_pub import EmqxPublisher  # ✅ 올바른 절대 import
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

def main():
    pub = EmqxPublisher()
    pub.start()
    # 메시지 발행 예시
    message = {
        "robot_id": "robot2",
        "status": "parking_complete"
    }
    pub.publish(message)
    pub.stop()