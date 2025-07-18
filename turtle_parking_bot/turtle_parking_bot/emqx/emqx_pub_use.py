try:
    # ROS2 실행용 (패키지로 설치된 경우)
    from turtle_parking_bot.emqx.emqx_pub import EmqxPublisher
except ImportError:
    # VS Code에서 직접 실행 (로컬 모듈 import)
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