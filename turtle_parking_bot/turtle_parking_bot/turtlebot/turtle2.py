#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import yaml
import time
import threading
import json
import signal

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from rclpy.node import Node
from pathlib import Path
from dotenv import load_dotenv

# 로컬 환경/ROS 패키지 구분 import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from turtle_parking_bot.turtlebot.turtlefunction import TurtleFunction
except ImportError:
    from turtlebot.turtlefunction import TurtleFunction


class ParkingSpotManager(Node):
    def __init__(self):
        super().__init__('parking_spot_manager')
        self.parking_spot = None
        self.lock = threading.Lock()
        self.signal_received = threading.Event()
        self.subscriber = None
        self.mqtt_connected = False
        
        # MQTT 연결을 안전하게 시작
        self._setup_mqtt()

    def _setup_mqtt(self):
        """MQTT 설정을 안전하게 수행"""
        try:
            self.get_logger().info("🔌 MQTT 연결 시도 중...")
            self.subscriber = TurtleFunction.emqx_run(self.my_callback)
            self.mqtt_connected = True
            self.get_logger().info("✅ MQTT 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 연결 실패: {e}")
            self.mqtt_connected = False

    def my_callback(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            
            self.get_logger().info(f"📨 MQTT 메시지 수신: {data}")

            if data.get("type") == "start":
                zone_id = data.get("zone_id")
                if zone_id:
                    self.update_parking_spot(zone_id)
                    if self.get_parking_spot() is not None:
                        self.signal_received.set()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 콜백 에러: {e}")

    def update_parking_spot(self, spot_name):
        coords = get_parking_spot_map_coord(spot_name)
        if coords is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            with self.lock:
                self.parking_spot = pose
            self.get_logger().info(f'🅿️ 수신된 주차 위치 "{spot_name}" → 좌표: x={coords[0]:.2f}, y={coords[1]:.2f}')
        else:
            self.get_logger().warn(f'❌ 유효하지 않은 주차 위치: "{spot_name}"')

    def get_parking_spot(self):
        with self.lock:
            return self.parking_spot

    def reset_signal(self):
        self.signal_received.clear()


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


def wait_for_parking_spot(parking_manager, timeout=30):
    """주차 위치 수신 대기"""
    print(f"📍 주차 위치 수신 대기 중... (최대 {timeout}초)")
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if parking_manager.signal_received.is_set():
            parking_spot = parking_manager.get_parking_spot()
            if parking_spot is not None:
                print(f"✅ 주차 위치 수신 완료")
                return parking_spot
        time.sleep(0.1)
    
    print("❌ 주차 위치 수신 타임아웃")
    return None


class TurtleBotController:
    def __init__(self, namespace='/robot2'):
        self.namespace = namespace
        self.navigator = None
        self.parking_manager = None
        self.executor = None
        self.executor_thread = None
        self.running = False

    def initialize(self):
        """시스템 초기화"""
        try:
            print("🚀 TurtleBot 컨트롤러 초기화 중...")
            
            # Navigator 생성
            self.navigator = TurtleBot4Navigator(namespace=self.namespace)
            
            # ParkingSpotManager 생성
            self.parking_manager = ParkingSpotManager()
            self.parking_manager.reset_signal()
            
            # MultiThreadedExecutor 설정
            self.executor = MultiThreadedExecutor(num_threads=4)
            self.executor.add_node(self.parking_manager)
            
            # Executor를 별도 스레드에서 실행
            self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
            self.executor_thread.start()
            self.running = True
            
            print("✅ 시스템 초기화 완료")
            time.sleep(2)  # 초기화 안정화 대기
            
            return True
            
        except Exception as e:
            print(f"❌ 초기화 실패: {e}")
            return False

    def _run_executor(self):
        """Executor 실행 (별도 스레드)"""
        try:
            while self.running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"❌ Executor 실행 중 오류: {e}")

    def wait_for_navigation_complete(self, timeout=90, action_name="네비게이션"):
        """네비게이션 완료 대기"""
        print(f"⏳ {action_name} 완료 대기 중... (최대 {timeout}초)")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                if self.navigator.isTaskComplete():
                    print(f"✅ {action_name} 완료")
                    return True
                time.sleep(0.1)
            except Exception as e:
                print(f"⚠️ 상태 확인 중 오류: {e}")
                time.sleep(0.5)
        
        print(f"⏰ {action_name} 타임아웃 ({timeout}초)")
        return False

    def set_initial_pose(self):
        """초기 위치 설정"""
        try:
            initial_pose = load_pose_from_yaml(
                os.path.expanduser('~/rokey_ws/maps/tb2_initial_pose.yaml'), 
                'initial_pose'
            )
            initial_pose.header.frame_id = 'map'
            self.navigator.setInitialPose(initial_pose)
            print("✅ 초기 위치 설정 완료")
            return True
        except Exception as e:
            print(f"⚠️ 초기 위치 설정 실패: {e}")
            return False

    def wait_nav2_active(self):
        """Nav2 활성화 대기"""
        try:
            print("📡 Nav2 활성화 대기 중...")
            self.navigator.waitUntilNav2Active()
            print("✅ Nav2 활성화 완료")
            return True
        except Exception as e:
            print(f"❌ Nav2 활성화 실패: {e}")
            return False

    def undock(self):
        """언도킹 실행"""
        try:
            print("🔄 언도킹 시작...")
            self.navigator.undock()
            # 언도킹은 완료 확인이 다를 수 있으므로 시간 대기
            time.sleep(5)
            print("✅ 언도킹 완료")
            return True
        except Exception as e:
            print(f"⚠️ 언도킹 실패: {e}")
            return False

    def go_to_parking_spot(self):
        """주차 위치로 이동"""
        # 주차 위치 수신 대기
        parking_pose = wait_for_parking_spot(self.parking_manager, timeout=30)
        if parking_pose is None:
            return False

        try:
            print("🚗 주차 위치로 이동 시작...")
            self.navigator.goToPose(parking_pose)
            return self.wait_for_navigation_complete(timeout=120, action_name="주차 위치 이동")
        except Exception as e:
            print(f"❌ 주차 위치 이동 실패: {e}")
            return False

    def go_to_pre_dock(self):
        """Pre-dock 위치로 이동"""
        try:
            pre_dock_pose = load_pose_from_yaml(
                os.path.expanduser('~/rokey_ws/maps/tb2_pre_dock_pose.yaml'), 
                'pre_dock_pose'
            )
            print("🔄 Pre-dock 위치로 이동 시작...")
            self.navigator.goToPose(pre_dock_pose)
            return self.wait_for_navigation_complete(timeout=90, action_name="Pre-dock 이동")
        except Exception as e:
            print(f"⚠️ Pre-dock 위치 이동 실패: {e}")
            return False

    def dock(self):
        """도킹 실행"""
        try:
            print("🔌 도킹 시작...")
            self.navigator.dock()
            # 도킹 완료 대기 (도킹은 시간이 오래 걸릴 수 있음)
            time.sleep(10)
            print("✅ 도킹 완료")
            return True
        except Exception as e:
            print(f"❌ 도킹 실패: {e}")
            return False

    def shutdown(self):
        """시스템 종료"""
        print("🔚 시스템 종료 중...")
        self.running = False
        
        if self.executor:
            try:
                self.executor.shutdown()
            except:
                pass
        
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2)


def main():
    # Signal handler for graceful shutdown
    def signal_handler(signum, frame):
        print("\n🛑 종료 신호 수신")
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # ROS2 초기화
    rclpy.init()
    
    controller = TurtleBotController()
    
    try:
        # 1. 시스템 초기화
        if not controller.initialize():
            return

        # 2. 초기 위치 설정
        controller.set_initial_pose()

        # 3. Nav2 활성화 대기
        if not controller.wait_nav2_active():
            return

        # 4. 언도킹
        if not controller.undock():
            print("⚠️ 언도킹에 실패했지만 계속 진행합니다.")

        # 5. 주차 위치로 이동
        if not controller.go_to_parking_spot():
            print("❌ 주차 위치 이동 실패")
            return

        print("🎯 주차 완료! 이제 충전 스테이션으로 복귀합니다.")

        # 6. Pre-dock 위치로 이동
        if not controller.go_to_pre_dock():
            print("⚠️ Pre-dock 위치 이동에 실패했지만 도킹을 시도합니다.")

        # 7. 도킹
        if controller.dock():
            print("🎉 모든 작업 완료! 로봇이 충전 스테이션에 도킹되었습니다.")
        else:
            print("❌ 도킹 실패")

    except Exception as e:
        print(f"❌ 예상치 못한 오류 발생: {e}")
    finally:
        controller.shutdown()
        rclpy.shutdown()
        print("👋 프로그램 종료")


if __name__ == '__main__':
    main()