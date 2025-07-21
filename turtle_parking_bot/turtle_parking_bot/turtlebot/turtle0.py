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

try:
    from turtle_parking_bot.turtlebot.turtlefunction import TurtleFunction
except ImportError:
    from turtlebot.turtlefunction import TurtleFunction

from std_msgs.msg import String
import json

class ParkingSpotManager(Node):
    def __init__(self,controller):
        super().__init__('parking_spot_manager')

        self.controller = controller

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

    # def my_callback(self, client, userdata, msg):
    #     try:
    #         payload = msg.payload.decode()
    #         data = json.loads(payload)
    #         self.get_logger().info(f"📨 MQTT 메시지 수신: {data}")
    #         if data.get("type") == "start":
    #             zone_id = data.get("zone_id")
    #             if zone_id:
    #                 self.update_parking_spot(zone_id)
    #                 if self.get_parking_spot():
    #                     self.signal_received.set()
    #     except Exception as e:
    #         self.get_logger().error(f"❌ MQTT 콜백 에러: {e}")

    def my_callback(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            # self.get_logger().info(f"📨 MQTT 메시지 수신임: {data}")

            # self.get_logger().info(f"📨 메세지 타입: ")

            # ✅ id가 0이 아니면 무시
            if data.get("id") != 'robot0':
                self.get_logger().info(f"🙅 ID가 0이 아님: {data.get('id')} → 무시됨")
                return

            msg_type = data.get("type")

            self.get_logger().info(f"📨 현재상태 : {self.controller.status} ")
            # self.get_logger().info(f"📨 메세지 타입: {msg_type} ")

            if msg_type == "start" and self.controller.status == "idle":
                zone_id = data.get("zone_id")
                if zone_id:
                    self.update_parking_spot(zone_id)
                    if self.get_parking_spot():
                        self.controller.update_status("navigating")
                        self.signal_received.set()

            elif msg_type == "cancel":
                self.controller.update_status("cancelled")

            elif msg_type == "pause":
                self.controller.update_status("paused")

            elif msg_type == "status":
                self.controller.update_status(self.controller.status)

            # else:
            #     self.get_logger().warn(f"⚠️ 알 수 없는 type: {msg_type}")
            #     self.controller.update_status("unknown_command")

        except Exception as e:
            self.get_logger().error(f"❌ MQTT 콜백 에러: {e}")
            # self.controller.update_status("error")



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
        "A1": (-2.91, -0.178),
        "A2": (-2.94, -0.569),
        "A3": (-2.96, -1.04),
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
        self.status = 'idle'
        self.running = False
        self.main_loop_thread = None  # 🔹 상태 처리 루프 쓰레드

    def send_unity_message(self, text: str):
        payload = {
            "type": "chatter",
            "msg": {
                "data": text
            }
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.unity_publisher.publish(msg)
        self.manager.get_logger().info(f"📤 Unity로 전송됨: {msg.data}")
    # def send_unity_message(self, text: str):
    #     payload = {
    #         "type": "chatter",
    #         "msg": {
    #             "data": text
    #         }
    #     }
    #     msg = String()
    #     msg.data = json.dumps(payload)

    #     def _safe_publish():
    #         try:
    #             self.unity_publisher.publish(msg)
    #             self.manager.get_logger().info(f"📤 Unity로 전송됨: {msg.data}")
    #         except Exception as e:
    #             self.manager.get_logger().error(f"❌ Unity publish 실패: {e}")

    #     # 안전하게 ROS2 이벤트 큐로 위임
    #     # self.manager.get_clock().call_soon_threadsafe(_safe_publish)
    #     self.executor.call_soon_threadsafe(_safe_publish)

    def init_unity_publisher(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.unity_publisher = self.manager.create_publisher(String, '/chatter', qos)
        self.manager.get_logger().info("📡 Unity 퍼블리셔 초기화 완료")

    def start_main_loop(self):
        self.main_loop_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_loop_thread.start()

    def _main_loop(self):
      
        self.manager.get_logger().info("🧠 메인 루프 시작")
        while self.running and rclpy.ok():
           
            self.manager.get_logger().info(f"현재 상태: {self.status}")

            if self.status == "navigating":
                self.send_unity_message("Start") 
                self.manager.get_logger().info("🚗 인계하러 가는 중")
             
                time.sleep(10) 
                if self.go_to_handover():
                    time.sleep(18) 
                    self.manager.get_logger().info("✅ 인계위치 도착") 
                    #
                    self.update_status('handover')

                else:
                    self.manager.get_logger().info("✅ 인계위치 도착 실패") 
              

            elif  self.status == "handover":
              
              
                self.send_unity_message("B2") 
                self.manager.get_logger().info("✅ 인계시작") 
                time.sleep(5) 
                self.send_unity_message("B3") 
                time.sleep(5) 
                self.go_to_parking_spot()
                time.sleep(5)
                self.send_unity_message("B4") 
                time.sleep(5)
                self.update_status('predock')


            elif  self.status == "predock":
                self.manager.get_logger().info("✅ 도킹 하러 가는중") 
                self.go_to_pre_dock()
                self.dock()
                # self.manager.get_logger().info("✅ 도킹 완료") 
                self.update_status('end')


            elif  self.status == "idle":
                print("🟡 대기 중")

            time.sleep(0.5)


    def initialize(self):
        print("🚀 컨트롤러 초기화 중...")
        self.navigator = TurtleBot4Navigator(namespace=self.namespace)
        self.manager = ParkingSpotManager(self)
        self.manager.reset_signal()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.manager)
        self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
        self.executor_thread.start()
        self.running = True
        self.status == "idle"
        self.init_unity_publisher()  # ✅ 추가
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
        pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_parking_pose.yaml'), 'parking_pose')
        if not pose:
            return False
        try:
            print("🚗 주차 위치로 이동...")
            self.navigator.goToPose(pose)
            return self._wait_for_nav_complete("주차 위치 이동")
        except Exception as e:
            print(f"❌ 주차 실패: {e}")
            return False
        
    def go_to_handover(self):
        pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_handover_pose.yaml'), 'handover_pose')
        if not pose:
            return False
        try:
            self.manager.get_logger().info("🚗 주차 위치로 이동...")
            self.navigator.goToPose(pose)
            return self._wait_for_nav_complete("주차 위치 이동")
        except Exception as e:
            self.manager.get_logger().info(f"❌ 주차 실패: {e}")
            return False
        # try:
        #     pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_handover_pose.yaml'), 'handoverpose')
        #     self.manager.get_logger().info("🚙 Pre-dock 이동 중...")
        #     self.navigator.goToPose(pose)
        #     return self._wait_for_nav_complete("Wait For 이동")
        # except Exception as e:
        #     self.manager.get_logger().info(f"⚠️ Pre-dock 실패: {e}")
        #     return False
          

    def go_to_pre_dock(self):
        try:
            pose = load_pose_from_yaml(os.path.expanduser('~/rokey_ws/maps/tb0_pre_dock_pose.yaml'), 'pre_dock_pose')
            # self.manager.get_logger.info("🚙 Pre-dock 이동 중...")
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
        self.manager.get_logger().info(f"⏳ {name} 완료 대기 중...")
        start = time.time()
        while time.time() - start < timeout:
            if self.navigator.isTaskComplete():
                print(f"✅ {name} 완료")
                return True
            time.sleep(0.1)
        print(f"⏰ {name} 타임아웃")
        return False
    
    def update_status(self, new_status: str):
        self.status = new_status



# def main():
#     def signal_handler(sig, frame):
#         print("\n🛑 종료 신호 수신됨")
#         if 'controller' in locals():
#             controller.shutdown()
#         rclpy.shutdown()
#         sys.exit(0)

#     signal.signal(signal.SIGINT, signal_handler)
#     rclpy.init()
#     controller = TurtleBotController()

#     while rclpy.ok():
#         print('동작중')
#         if controller.status == "idle":re
#             print("아무것도 안하고 있음")
#         if controller.status == "navigating":
#             print("인계하러 가는중")
#         time.sleep(0.5)


#     print("🟡 대기 중1: MQTT에서 'start' 명령을 기다립니다.")
#     try:
#         if not controller.initialize():
#             return
#         controller.set_initial_pose()
#         if not controller.wait_nav2_active():
#             return
#         if not controller.undock():
#             print("⚠️ 언도킹 실패 (계속 진행)")

#         # ✅ status가 "start"일 때까지 대기
#         print("🟡 대기 중2: MQTT에서 'start' 명령을 기다립니다.")
   

#         if not controller.go_to_parking_spot():
#             print("❌ 주차 실패")
#             return

#         print("🎯 주차 완료. 충전 위치로 복귀 중...")
#         if not controller.go_to_pre_dock():
#             print("⚠️ Pre-dock 실패 (도킹 시도)")
#         if controller.dock():
#             print("🎉 모든 작업 완료! 충전 도킹 성공")
#         else:
#             print("❌ 도킹 실패")

#         # ✅ 작업 완료 후 다시 idle로 상태 리셋
#         controller.update_status("idle")
#         print("🔁 상태를 'idle'로 초기화하고 다시 대기합니다.")
#     except Exception as e:
#         print(f"❌ 오류 발생: {e}")
#     finally:
#         controller.shutdown()
#         rclpy.shutdown()
#         print("👋 프로그램 종료")

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
    print('aaaaaaaaaaaaaaaaaaaaaa')
    try:
        print('bbbbbbbbbbbbbbbbbbbbbbbb')
        if not controller.initialize():
            print('notinit')
            return
        
        # if not controller.navigator.isDocked():
        #     print("🚫 도킹 상태가 아니므로 시작하지 않습니다.")
        #     controller.shutdown()
        #     rclpy.shutdown()
        #     return

        controller.set_initial_pose()
        if not controller.wait_nav2_active():
            print('ccccccccccccccc')
            return
        if not controller.undock():
            print("⚠️ 언도킹 실패 (계속 진행)")

        print('dddddddddddddddddddd')
        # 🔄 메인 루프 시작 (쓰레드로)
        controller.start_main_loop()

        # 메인 쓰레드는 대기만
        while rclpy.ok():
            time.sleep(1)

    except Exception as e:
        print(f"❌ 오류 발생: {e}")
    finally:
        controller.shutdown()
        rclpy.shutdown()
        print("👋 프로그램 종료")




if __name__ == '__main__':
    main()
