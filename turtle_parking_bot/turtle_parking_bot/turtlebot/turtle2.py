#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
import time
import threading
import json
import signal

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header, Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from rclpy.node import Node
from pathlib import Path
from dotenv import load_dotenv

try:
    from turtle_parking_bot.turtlebot.canon_node import CanonNode
except ImportError:
    from canon_node import CanonNode

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
        
        self._setup_mqtt()

    def _setup_mqtt(self):
        try:
            self.get_logger().info("MQTT 연결 시도 중...")
            self.subscriber = TurtleFunction.emqx_run(self.my_callback)
            self.mqtt_connected = True
            self.get_logger().info("MQTT 연결 성공")
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
            self.mqtt_connected = False

    def my_callback(self, client, userdata, msg):
        try:
            payload = msg.payload.decode().strip()
            if not payload:
                return
            
            try:
                data = json.loads(payload)
                if not isinstance(data, dict):
                    return
            except json.JSONDecodeError:
                return
            
            self.get_logger().info(f"파싱된 MQTT 메시지: {data}")

            if data.get("type") == "start":
                zone_id = data.get("zone_id")
                if zone_id:
                    self.update_parking_spot(zone_id)
                    if self.get_parking_spot() is not None:
                        self.signal_received.set()
        except Exception as e:
            self.get_logger().error(f"MQTT 콜백 에러: {e}")

    def update_parking_spot(self, spot_name):
        pose = load_pose_from_config(spot_name)
        if pose is not None:
            pose.header.stamp = self.get_clock().now().to_msg()
            with self.lock:
                self.parking_spot = pose
            self.get_logger().info(f'주차 위치 "{spot_name}" 좌표 설정 완료')

    def get_parking_spot(self):
        with self.lock:
            return self.parking_spot

    def reset_signal(self):
        self.signal_received.clear()
        with self.lock:
            self.parking_spot = None 


# 주차 구역 이름(A1, B2, initial 등)에 따라 PoseStamped 반환
def load_pose_from_config(spot_name: str) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header = Header()
    pose_stamped.header.frame_id = 'map'

    # A구역은 하드코딩된 좌표
    if spot_name in ["A1", "A2", "A3"]:
        parking_map = {
            "A1": ((0.00814, 0.615), (0.0, 0.0, 0.67, 0.74)),
            "A2": ((-0.16, 0.626), (0.0, 0.0, 0.67, 0.74)),
            "A3": ((-0.8, 0.5), (0.0, 0.0, 0.67, 0.74)),
        }
        pos, ori = parking_map[spot_name]
        pose_stamped.pose.position.x, pose_stamped.pose.position.y = pos
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w = ori

    # B구역 또는 초기 위치 등 YAML에서 로드
    else:
        if spot_name in ["B1", "B2", "B3"]:
            yaml_path = os.path.expanduser('~/rokey_ws/maps/handover_point.yaml')
            key = 'handover_pose'
        elif spot_name == "initial":
            yaml_path = os.path.expanduser('~/rokey_ws/maps/tb2_initial_pose.yaml')
            key = 'initial_pose'
        elif spot_name == "pre_dock":
            yaml_path = os.path.expanduser('~/rokey_ws/maps/tb2_pre_dock_pose.yaml')
            key = 'pre_dock_pose'
        else:
            print(f"❌ 알 수 없는 spot_name: {spot_name}")
            return None

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            pose = data[key]['pose']
            pos = pose['position']
            ori = pose['orientation']

            pose_stamped.pose.position.x = float(pos['x'])
            pose_stamped.pose.position.y = float(pos['y'])
            pose_stamped.pose.position.z = float(pos.get('z', 0.0))
            pose_stamped.pose.orientation.x = float(ori['x'])
            pose_stamped.pose.orientation.y = float(ori['y'])
            pose_stamped.pose.orientation.z = float(ori['z'])
            pose_stamped.pose.orientation.w = float(ori['w'])

        except Exception as e:
            print(f"❌ YAML 로드 오류: {e}")
            return None

    return pose_stamped


def wait_for_parking_spot(parking_manager, timeout=30):
    print(f"주차 위치 수신 대기 중... (최대 {timeout}초)")
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if parking_manager.signal_received.is_set():
            parking_spot = parking_manager.get_parking_spot()
            if parking_spot is not None:
                return parking_spot
        time.sleep(0.1)
    
    return None


class TurtleBotController:
    def __init__(self, namespace='/robot2'):
        self.namespace = namespace
        self.navigator = None
        self.parking_manager = None
        self.canon_node = None
        self.audio_stop_publisher = None
        self.executor = None
        self.executor_thread = None
        self.running = False

    def wait_for_start_signal(self, timeout=None):
        print("'start' 메시지 수신 대기 중...")
        if self.parking_manager.signal_received.wait(timeout=timeout):
            print("'start' 메시지 수신됨! 동작 시작")
            return True
        return False

    def initialize(self):
        try:
            print("TurtleBot 컨트롤러 초기화 중...")
            
            self.navigator = TurtleBot4Navigator(namespace=self.namespace)
            self.parking_manager = ParkingSpotManager()
            self.parking_manager.reset_signal()
            
            print("Canon Node 초기화 중...")
            self.canon_node = CanonNode()
            
            self.audio_stop_publisher = self.parking_manager.create_publisher(
                Bool, '/robot2/audio_stop', 10
            )
            
            # MultiThreadedExecutor로 두 노드 동시 실행
            self.executor = MultiThreadedExecutor(num_threads=4)
            self.executor.add_node(self.parking_manager)
            self.executor.add_node(self.canon_node)
            
            self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
            self.executor_thread.start()
            self.running = True
            
            print("시스템 초기화 완료")
            time.sleep(2)
            
            return True
            
        except Exception as e:
            print(f"초기화 실패: {e}")
            return False

    def _run_executor(self):
        try:
            while self.running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Executor 실행 중 오류: {e}")

    def start_music(self):
        try:
            print("캐논 연주 시작!")
            if hasattr(self.canon_node, 'should_stop'):
                self.canon_node.should_stop = False  # 🔥 여기 중요

            if hasattr(self.canon_node, 'start_continuous_playing'):
                self.canon_node.start_continuous_playing()
            return True
        except Exception as e:
            print(f"음악 시작 실패: {e}")
            return False

    def stop_music(self):
        try:
            print("캐논 연주 중단")
            
            if hasattr(self.canon_node, 'should_stop'):
                self.canon_node.should_stop = True
            
            stop_msg = Bool()
            stop_msg.data = True
            self.audio_stop_publisher.publish(stop_msg)
            time.sleep(1.0)
            return True
        except Exception as e:
            print(f"음악 정지 실패: {e}")
            return False

    def wait_for_navigation_complete(self, timeout=90, action_name="네비게이션"):
        print(f"{action_name} 완료 대기 중... (최대 {timeout}초)")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                if self.navigator.isTaskComplete():
                    result = self.navigator.getResult()
                    if result and hasattr(result, 'result'):
                        if result.result:
                            print(f"{action_name} 성공")
                            return True
                        else:
                            print(f"{action_name} 실패")
                            return False
                    else:
                        print(f"{action_name} 완료")
                        return True
                time.sleep(0.1)  # 더 빠른 상태 체크를 위해 0.1초로 단축
            except Exception as e:
                print(f"상태 확인 중 오류: {e}")
                time.sleep(0.5)
        
        print(f"{action_name} 타임아웃 ({timeout}초)")
        return False

    def set_initial_pose(self):
        try:
            initial_pose = load_pose_from_config("initial")
            if initial_pose is None:
                raise ValueError("초기 위치 정보를 불러올 수 없습니다.")

            initial_pose.header.stamp = self.parking_manager.get_clock().now().to_msg()
            self.navigator.setInitialPose(initial_pose)
            print("초기 위치 설정 완료")
            time.sleep(1)
            return True
        except Exception as e:
            print(f"초기 위치 설정 실패: {e}")
            return False

    def wait_nav2_active(self):
        try:
            print("Nav2 활성화 대기 중...")
            time.sleep(5)  # Nav2 준비 시간
            print("Nav2 활성화 완료")
            return True
        except Exception as e:
            print(f"Nav2 활성화 실패: {e}")
            return True

    def undock(self):
        try:
            print("언도킹 시작...")
            self.navigator.undock()
            
            # 완료 상태를 실시간으로 확인하면서 대기
            result = self.wait_for_navigation_complete(timeout=30, action_name="언도킹")
            
            if result:
                print("언도킹 성공")
                time.sleep(1)  # 안정화를 위한 최소한의 대기
                return True
            else:
                print("언도킹 타임아웃 또는 실패")
                # 실패해도 로봇이 어느 정도 움직였을 가능성을 고려해 짧은 대기
                time.sleep(2)
                return False
        except Exception as e:
            print(f"언도킹 실패: {e}")
            return False

    def go_to_parking_spot(self):
        parking_pose = wait_for_parking_spot(self.parking_manager, timeout=30)
        if parking_pose is None:
            existing_spot = self.parking_manager.get_parking_spot()
            if existing_spot is not None:
                parking_pose = existing_spot
            else:
                print("주차 위치를 받지 못했습니다")
                return False

        try:
            spot_name = None
            with self.parking_manager.lock:
                # spot_name 추정 (pose 기반이므로 정확하진 않음, 추가 구조 필요시 개선)
                for name in ["A1", "A2", "A3"]:
                    test_pose = load_pose_from_config(name)
                    if test_pose and test_pose.pose.position.x == parking_pose.pose.position.x:
                        spot_name = name
                        break

            if spot_name in ["A1", "A2", "A3"]:
                print(f"🛣️ {spot_name} 경유지 → 주차지점으로 이동")

                # === 1단계: waypoint 이동 ===
                waypoint_pose = PoseStamped()
                waypoint_pose.header.frame_id = 'map'
                waypoint_pose.header.stamp = self.parking_manager.get_clock().now().to_msg()

                # ✅ 사용자 지정 (임의로 수정 가능)
                waypoint_pose.pose.position.x = -1.1
                waypoint_pose.pose.position.y = 0.8
                waypoint_pose.pose.position.z = 0.0
                waypoint_pose.pose.orientation.w = 1.0  # 단순한 정면

                print("📍 경유지로 먼저 이동 중...")
                self.navigator.goToPose(waypoint_pose)
                success = self.wait_for_navigation_complete(timeout=60, action_name="경유지 이동")
                if not success:
                    print("❌ 경유지 이동 실패")
                    return False

                time.sleep(1)

            # === 2단계: 최종 주차 위치 이동 ===
            parking_pose.header.stamp = self.parking_manager.get_clock().now().to_msg()
            print(f"🚗 주차 위치로 이동: x={parking_pose.pose.position.x:.2f}, y={parking_pose.pose.position.y:.2f}")
            self.navigator.goToPose(parking_pose)
            return self.wait_for_navigation_complete(timeout=120, action_name="주차 이동")

        except Exception as e:
            print(f"주차 위치 이동 실패: {e}")
            return False


    def go_to_pre_dock(self):
        try:
            pre_dock_pose = load_pose_from_config("pre_dock")
            if pre_dock_pose is None:
                raise ValueError("Pre-dock 위치 정보를 불러올 수 없습니다.")

            pre_dock_pose.header.stamp = self.parking_manager.get_clock().now().to_msg()
            print("Pre-dock 위치로 이동...")
            self.navigator.goToPose(pre_dock_pose)
            return self.wait_for_navigation_complete(timeout=90, action_name="Pre-dock 이동")
        except Exception as e:
            print(f"Pre-dock 이동 실패: {e}")
            return False


    def dock(self):
        try:
            print("도킹 시작...")
            self.navigator.dock()
            
            # 완료 상태를 실시간으로 확인하면서 대기
            result = self.wait_for_navigation_complete(timeout=30, action_name="도킹")
            
            if result:
                print("도킹 성공")
                return True
            else:
                print("도킹 타임아웃 또는 실패")
                time.sleep(2)  # 실패해도 충분한 시간을 줌
                return True  # 도킹은 실패해도 True 리턴 (기존 로직 유지)
        except Exception as e:
            print(f"도킹 실패: {e}")
            return False

    def shutdown(self):
        print("시스템 종료 중...")
        
        try:
            self.navigator.cancelTask()
        except:
            pass
        
        self.stop_music()
        self.running = False
        
        if self.executor:
            try:
                self.executor.shutdown()
            except:
                pass
        
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2)


# def main():
#     def signal_handler(signum, frame):
#         print("\n종료 신호 수신")
#         if 'controller' in locals():
#             controller.shutdown()
#         rclpy.shutdown()
#         sys.exit(0)

#     signal.signal(signal.SIGINT, signal_handler)
#     rclpy.init()

#     controller = TurtleBotController()

#     try:
#         if not controller.initialize():
#             return

#         if not controller.wait_for_start_signal(timeout=300):
#             print("시작 신호 수신 실패")
#             return

#         print("로봇 동작 시퀀스 시작")
        
#         controller.set_initial_pose()
#         controller.wait_nav2_active()
        
#         # controller.start_music()  # 소리 나옴
#         time.sleep(1)
        
#         controller.undock()
#         time.sleep(2)
        
#         if not controller.go_to_parking_spot():
#             controller.stop_music()
#             return
            
#         print("주차 완료! 충전 스테이션으로 복귀")
#         time.sleep(5)
        
#         controller.go_to_pre_dock()
#         controller.stop_music()
#         time.sleep(1)
        
#         controller.dock()
#         print("모든 작업 완료")
            
#     except Exception as e:
#         print(f"오류 발생: {e}")
#     finally:
#         controller.shutdown()
#         rclpy.shutdown()

def main():
    def signal_handler(signum, frame):
        print("\n종료 신호 수신")
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
        controller.wait_nav2_active()

        while rclpy.ok():
            print("\n📥 [대기] start 메시지 수신 대기 중...")
            if not controller.wait_for_start_signal(timeout=600):  # 10분 대기
                print("⚠️ 시작 신호를 받지 못했습니다. 다시 대기합니다.")
                continue

            controller.parking_manager.reset_signal()  # 메시지 플래그 리셋
            print("\n🚗 주차 루틴 시작")

            time.sleep(1)
            controller.undock()
            time.sleep(2)

            if not controller.go_to_parking_spot():
                controller.stop_music()
                print("❌ 주차 실패")
                continue  # 다음 start 대기

            print("✅ 주차 완료, 도킹 복귀 시작")
            time.sleep(3)

            controller.go_to_pre_dock()
            controller.stop_music()
            time.sleep(1)

            controller.dock()
            print("✅ 도킹 완료")

            # 다음 start 메시지를 기다리도록 루프 계속
            time.sleep(2)

    except Exception as e:
        print(f"🚨 오류 발생: {e}")
    finally:
        controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()