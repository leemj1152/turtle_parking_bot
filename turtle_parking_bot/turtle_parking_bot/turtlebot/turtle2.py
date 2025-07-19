#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import rclpy
import yaml

import time
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header, String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from rclpy.node import Node

# from turtle_parking_bot.emqx.emqx_sub import EmqxSubscriber
# from turtle_parking_bot.turtlebot import emqx_run
from paho.mqtt import client as mqtt_client
import random
import json

from pathlib import Path
from dotenv import load_dotenv
# from turtle_parking_bot.turtlebot.turtlefunction import TurtleFunction
try:
    # ROS2 실행용 (패키지로 설치된 경우)
    from turtle_parking_bot.turtlebot.turtlefunction import TurtleFunction
except ImportError:
    # VS Code에서 직접 실행 (로컬 모듈 import)
    from turtlebot.turtlefunction import TurtleFunction


class ParkingSpotManager(Node):
    def __init__(self):
        super().__init__('parking_spot_manager')
        self.parking_spot = None
        self.lock = threading.Lock()
        self.signal_received = threading.Event()        
        self.subscriber = TurtleFunction.emqx_run(self.my_callback)
      
    def my_callback(self, client, userdata, msg):
        # try:
        payload = msg.payload.decode()
        print(f"[👋 custom] message: {payload}")
        data = json.loads(payload)

        if data.get("type") == "start":
            zone_id = data.get("zone_id")
            if zone_id:
                self.update_parking_spot(zone_id)
                if self.get_parking_spot() is not None:
                    self.signal_received.set()

        #     if data.get("type") == "start":
        #         zone_id = data.get("zone_id")
        #         if zone_id:
        #             self.update_parking_spot(zone_id)
        #             self.signal_received.set()  # ✅ 이벤트 플래그 설정                    # self.start_signal_received.set()
        # except Exception as e:
        #     print(f"[❌ MQTT 처리 에러] {e}")

    def update_parking_spot(self, spot_name):
        coords = get_parking_spot_map_coord(spot_name)
        if coords is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.orientation.w = 1.0
            with self.lock:
                self.parking_spot = pose
            self.get_logger().info(f'🅿️ 수신된 주차 위치 "{spot_name}" → 좌표: x={coords[0]:.2f}, y={coords[1]:.2f}')
        else:
            self.get_logger().warn(f'❌ 유효하지 않은 주차 위치: "{spot_name}"')

    def get_parking_spot(self):
        with self.lock:
            return self.parking_spot


def get_parking_spot_map_coord(spot_name: str):
    parking_map = {
        "A1": (0.00814, 0.615),
        "A2": (-1.04, 0.577),
        "A3": (-1.69, 0.528),
        "B1": (-2.91, -0.178),
        "B2": (-2.94, -0.569),
        "B3": (-2.96, -1.04)
    }
    return parking_map.get(spot_name, None)


def load_pose_from_yaml(yaml_path: str, key: str) -> PoseStamped:
    try:
        import yaml
        from geometry_msgs.msg import PoseStamped, Pose
        from std_msgs.msg import Header
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        pose_data = data[key]['pose']
        header_data = data[key].get('header', {'frame_id': 'map'})
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = header_data.get('frame_id', 'map')
        pose_stamped.pose = Pose()
        pose_stamped.pose.position.x = pose_data['position']['x']
        pose_stamped.pose.position.y = pose_data['position']['y']
        pose_stamped.pose.position.z = pose_data['position'].get('z', 0.0)
        pose_stamped.pose.orientation.x = pose_data['orientation']['x']
        pose_stamped.pose.orientation.y = pose_data['orientation']['y']
        pose_stamped.pose.orientation.z = pose_data['orientation']['z']
        pose_stamped.pose.orientation.w = pose_data['orientation']['w']
        return pose_stamped
    except Exception as e:
        print(f"❌ YAML 로딩 실패: {e}")
        raise


def wait_for_parking_spot(parking_manager, timeout=30):
    print(f"📡 주차 위치 대기 중... (최대 {timeout}초)")
    # start_time = time.time()
    # while time.time() - start_time < timeout:
        # rclpy.spin_once(parking_manager, timeout_sec=0.1)
    if parking_manager.signal_received.wait(timeout=timeout):
        parking_spot = parking_manager.get_parking_spot()
        if parking_spot is not None:
            print(f"   ✅ 주차 위치 수신: ({parking_spot.pose.position.x:.2f}, {parking_spot.pose.position.y:.2f})")
            return parking_spot
        time.sleep(0.1)
    print("   ⚠️ 주차 위치 수신 타임아웃")
    return None


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator(namespace='/robot2')
    parking_manager = ParkingSpotManager()

    parking_manager.signal_received.clear()



    # MQTT 구독 시작
    # mqtt_subscriber = MQTTSubscriber(on_message_callback=parking_manager.update_parking_spot)
    # mqtt_subscriber.start()

    print('aaaaaaaaaaaaaaaaaaaaaaa')

    try:
        # 초기 위치 설정
        try:
            initial_pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb2_initial_pose.yaml'), 'initial_pose')
            initial_pose.header.frame_id = 'map'
            navigator.setInitialPose(initial_pose)
        
            spin_thread = threading.Thread(target=rclpy.spin, args=(parking_manager,), daemon=True)
            spin_thread.start()

            print(f"   ✅ 초기 위치 설정 완료")
        except Exception as e:
            print(f"   ⚠️ 초기 위치 설정 실패: {e}")


        print(f"  Next Step")
        # undock
        try:
            navigator.undock()
            print("   ✅ Undocking 완료")
        except Exception as e:
            print(f"   ⚠️ Undocking 실패: {e}")

        # Nav2 활성화 대기
      
        nav2_ready = threading.Event()

        def wait_for_nav2():
            try:
                navigator.waitUntilNav2Active()
                nav2_ready.set()
            except Exception as e:
                print(f"   ❌ Nav2 활성화 실패: {e}")

        nav2_thread = threading.Thread(target=wait_for_nav2)
        nav2_thread.start()

        if nav2_ready.wait(timeout=60):
            print("   ✅ Nav2 활성화 완료")
        else:
            print("   ⚠️ Nav2 활성화 타임아웃")

        # MQTT로부터 주차 위치 수신 대기
        parking_pose = wait_for_parking_spot(parking_manager, timeout=30)
        if parking_pose is None:
            print("   ❌ 주차 위치 수신 실패, 종료")
            return

        # 주차 위치로 이동
        print("5️⃣ 주차 위치로 이동 중...")
        navigator.goToPose(parking_pose)

        timeout = 60
        elapsed = 0
        while not navigator.isTaskComplete() and elapsed < timeout:
            time.sleep(0.1)
            elapsed += 0.1

        if elapsed >= timeout:
            print("   ⚠️ 주차 위치 도착 타임아웃")
        else:
            print("   ✅ 주차 위치 도착 완료")

        # pre-dock 위치 이동
        print("6️⃣ Pre-dock 위치로 이동 중...")
        try:
            pre_dock_pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb2_pre_dock_pose.yaml'), 'pre_dock_pose')
            navigator.goToPose(pre_dock_pose)

            elapsed = 0
            while not navigator.isTaskComplete() and elapsed < timeout:
                time.sleep(0.1)
                elapsed += 0.1

            if elapsed >= timeout:
                print("   ⚠️ Pre-dock 위치 도착 타임아웃")
            else:
                print("   ✅ Pre-dock 위치 도착 완료")
        except Exception as e:
            print(f"   ⚠️ Pre-dock 위치 이동 실패: {e}")

        # 도킹 수행
        print("7️⃣ 도킹 수행 중...")
        try:
            navigator.dock()
            print("   ✅ 도킹 완료")
        except Exception as e:
            print(f"   ❌ 도킹 실패: {e}")

        print("✅ 네비게이션 완료")

    except Exception as e:
        print(f"❌ 예외 발생: {e}")
    finally:
        # mqtt_subscriber.stop()
        rclpy.shutdown()
        print("🏁 프로그램 종료")

if __name__ == '__main__':
    main()
