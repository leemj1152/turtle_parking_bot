# 다른 파일 예: controller_node.py

try:
    # ROS2 실행용 (패키지로 설치된 경우)
    from turtle_parking_bot.emqx.emqx_sub import EmqxSubscriber
except ImportError:
    # VS Code에서 직접 실행 (로컬 모듈 import)
    from emqx_sub import EmqxSubscriber

def my_callback(client, userdata, msg):
    print(f"[👋 custom] message: {msg.payload.decode()}")

def main():
    sub = EmqxSubscriber(callback=my_callback)
    sub.run()

if __name__ == '__main__':
    main()

