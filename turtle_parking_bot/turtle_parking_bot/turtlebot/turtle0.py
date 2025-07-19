#!/usr/bin/env python3
import sys
import os
import rclpy
import time
import json
import yaml
import threading
import signal

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

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
        self._setup_mqtt()

    def _setup_mqtt(self):
        try:
            self.get_logger().info("🔌 MQTT 연결 시도 중...")
            self.subscriber = TurtleFunction.emqx_run(self.my_callback)
            self.get_logger().info("✅ MQTT 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 연결 실패: {e}")

    def my_callback(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            self.get_logger().info(f"📨 MQTT 메시지 수신: {data}")
            if data.get("type") == "start":
                zone_id = data.get("zone_id")
                if zone_id:
                    self.update_parking_spot(zone_id)
                    if self.get_parking_spot():
                        self.signal_received.set()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 콜백 에러: {e}")

    def update_parking_spot(self, spot_name):
        coords = get_parking_spot_map_coord(spot_name)
        if coords:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.orientation.w = 1.0
            with self.lock:
                self.parking_spot = pose
            self.get_logger().info(f"🅿️ 주차 위치: {spot_name} → {coords}")
        else:
            self.get_logger().warn(f"❌ 유효하지 않은 주차 위치: {spot_name}")

    def get_parking_spot(self):
        with self.lock:
            return self.parking_spot

    def reset_signal(self):
        self.signal_received.clear()


def get_parking_spot_map_coord(spot_name: str):
    return {
        "A1": (0.00814, 0.615),
        "A2": (-1.04, 0.577),
        "A3": (-1.69, 0.528),
        "B1": (-2.91, -0.178),
        "B2": (-2.94, -0.569),
        "B3": (-2.96, -1.04)
    }.get(spot_name, None)


def load_pose_from_yaml(yaml_path: str, key: str) -> PoseStamped:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    pose_data = data[key]['pose']
    header_data = data[key].get('header', {'frame_id': 'map'})
    pose_stamped = PoseStamped()
    pose_stamped.header = Header(frame_id=header_data.get('frame_id', 'map'))
    pose_stamped.pose = Pose()
    pose_stamped.pose.position.x = pose_data['position']['x']
    pose_stamped.pose.position.y = pose_data['position']['y']
    pose_stamped.pose.orientation.x = pose_data['orientation']['x']
    pose_stamped.pose.orientation.y = pose_data['orientation']['y']
    pose_stamped.pose.orientation.z = pose_data['orientation']['z']
    pose_stamped.pose.orientation.w = pose_data['orientation']['w']
    return pose_stamped


def wait_for_parking_spot(manager, timeout=30):
    print("📍 주차 위치 수신 대기 중...")
    start = time.time()
    while time.time() - start < timeout:
        if manager.signal_received.is_set():
            return manager.get_parking_spot()
        time.sleep(0.1)
    print("❌ 주차 위치 수신 실패")
    return None


class TurtleBotController:
    def __init__(self, namespace='/robot0'):
        self.namespace = namespace
        self.navigator = None
        self.manager = None
        self.executor = None
        self.executor_thread = None
        self.running = False

    def initialize(self):
        print("🚀 컨트롤러 초기화 중...")
        self.navigator = TurtleBot4Navigator(namespace=self.namespace)
        self.manager = ParkingSpotManager()
        self.manager.reset_signal()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.manager)
        self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
        self.executor_thread.start()
        self.running = True
        time.sleep(2)
        print("✅ 초기화 완료")
        return True

    def _run_executor(self):
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def shutdown(self):
        print("🔚 종료 중...")
        self.running = False
        try:
            self.executor.shutdown()
        except:
            pass
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2)

    def set_initial_pose(self):
        try:
            pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_initial_pose.yaml'), 'initial_pose')
            pose.header.frame_id = 'map'
            self.navigator.setInitialPose(pose)
            print("✅ 초기 위치 설정 완료")
            return True
        except Exception as e:
            print(f"⚠️ 초기 위치 설정 실패: {e}")
            return False

    def wait_nav2_active(self):
        try:
            print("📡 Nav2 활성화 대기 중...")
            self.navigator.waitUntilNav2Active()
            print("✅ Nav2 활성화 완료")
            return True
        except Exception as e:
            print(f"❌ Nav2 활성화 실패: {e}")
            return False

    def undock(self):
        try:
            print("🔄 언도킹 시작...")
            self.navigator.undock()
            time.sleep(5)
            print("✅ 언도킹 완료")
            return True
        except Exception as e:
            print(f"⚠️ 언도킹 실패: {e}")
            return False

    def go_to_parking_spot(self):
        pose = wait_for_parking_spot(self.manager)
        if not pose:
            return False
        try:
            print("🚗 주차 위치로 이동...")
            self.navigator.goToPose(pose)
            return self._wait_for_nav_complete("주차 위치 이동")
        except Exception as e:
            print(f"❌ 주차 실패: {e}")
            return False

    def go_to_pre_dock(self):
        try:
            pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_pre_dock_pose.yaml'), 'pre_dock_pose')
            print("🚙 Pre-dock 이동 중...")
            self.navigator.goToPose(pose)
            return self._wait_for_nav_complete("Pre-dock 이동")
        except Exception as e:
            print(f"⚠️ Pre-dock 실패: {e}")
            return False

    def dock(self):
        try:
            print("🔌 도킹 시도 중...")
            self.navigator.dock()
            time.sleep(10)
            print("✅ 도킹 완료")
            return True
        except Exception as e:
            print(f"❌ 도킹 실패: {e}")
            return False

    def _wait_for_nav_complete(self, name="작업", timeout=90):
        print(f"⏳ {name} 완료 대기 중...")
        start = time.time()
        while time.time() - start < timeout:
            if self.navigator.isTaskComplete():
                print(f"✅ {name} 완료")
                return True
            time.sleep(0.1)
        print(f"⏰ {name} 타임아웃")
        return False


def main():
    def signal_handler(sig, frame):
        print("\n🛑 종료 신호 수신됨")
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    controller = TurtleBotController()

    try:
        if not controller.initialize():
            return
        controller.set_initial_pose()
        if not controller.wait_nav2_active():
            return
        if not controller.undock():
            print("⚠️ 언도킹 실패 (계속 진행)")
        if not controller.go_to_parking_spot():
            print("❌ 주차 실패")
            return
        print("🎯 주차 완료. 충전 위치로 복귀 중...")
        if not controller.go_to_pre_dock():
            print("⚠️ Pre-dock 실패 (도킹 시도)")
        if controller.dock():
            print("🎉 모든 작업 완료! 충전 도킹 성공")
        else:
            print("❌ 도킹 실패")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
    finally:
        controller.shutdown()
        rclpy.shutdown()
        print("👋 프로그램 종료")


if __name__ == '__main__':
    main()
