# import rclpy
# import emqx_pub
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import String
# from emqx_pub import Emqx
# class CentralController(Node):
#     def __init__(self):
#         super().__init__('central_controller')
#         self.empty_spots = []  
#         self.used_spots = set()
#         self.spot_sub = self.create_subscription(PoseStamped, '/parking/empty_spot', self.spot_callback, 10)
#         self.result_sub = self.create_subscription(String, '/robot/result', self.result_callback, 10)
#         self.command_pub = self.create_publisher(PoseStamped, '/robot/target_spot', 10)
#         self.waiting_for_result = False
#         # emqx_pub.run() 
#         client = emqx_pub.connect_mqtt()
#         client.loop_start()
#         message = {
#             "id": "0",
#             "location": "B3" # --> 구역반환
#         }
#         emqx_pub.publish(client, message)


#     def spot_callback(self, msg):
#         if self.waiting_for_result:
#             return  
#         key = (msg.pose.position.x, msg.pose.position.y)
#         if key in self.used_spots:
#             return
#         self.get_logger().info(f"빈 좌표 수신: {key}")
#         self.empty_spots.append(msg)
#         self.try_send_next()

#     def try_send_next(self):
#         while self.empty_spots:
#             next_spot = self.empty_spots.pop(0)  # 나중에 주차구역 순서 정해지면 따로 추가로 정리
#             key = (next_spot.pose.position.x, next_spot.pose.position.y)
#             if key not in self.used_spots:
#                 self.used_spots.add(key)
#                 self.command_pub.publish(next_spot)
#                 self.get_logger().info(f"로봇에게 좌표 전송: {key}")
#                 self.waiting_for_result = True
#                 break

#     def result_callback(self, msg):
#         result = msg.data
#         self.waiting_for_result = False
#         if result == 'success':
#             self.get_logger().info("주차 성공")
#             rclpy.shutdown()
#         elif result == 'fail':
#             self.get_logger().warn("주차 실패 → 좌표 재전송")
#             self.try_send_next()
#         else:
#             self.get_logger().warn("불필요한 좌표 전송")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CentralController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("중앙 제어 노드 종료")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import json
from emqx_pub import connect_mqtt, publish


# 1. 주차 구역 ID → 좌표 매핑 함수
def get_parking_spot_map_coord(spot_name: str):
    parking_map = {
        "A1": (2.96, 2.15),
        "A2": (2.5, 2.0),
        "B1": (3.2, 1.6),
        "B2": (3.5, 1.6),
        "B3": (3.81, 1.64),
    }
    return parking_map.get(spot_name, None)


# 2. parking spot subscriber (주차 ID → Pose 변환 저장)
class ParkingSpotSubscriber(Node):
    def __init__(self):
        super().__init__('parking_spot_subscriber')
        self.parking_spot = None
        self.subscription = self.create_subscription(
            String,
            '/parking/empty_spot_id',
            self.parking_spot_callback,
            10
        )
        self.get_logger().info('📡 /parking/empty_spot_id 구독 시작')

    def parking_spot_callback(self, msg):
        spot_name = msg.data.strip()
        coords = get_parking_spot_map_coord(spot_name)
        if coords is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.orientation.w = 1.0
            self.parking_spot = (spot_name, pose)
            self.get_logger().info(f'🅿️ 수신: {spot_name} → x={coords[0]:.2f}, y={coords[1]:.2f}')
        else:
            self.get_logger().warn(f'❌ 알 수 없는 구역 ID: "{spot_name}"')

    def get_parking_spot(self):
        return self.parking_spot


# 3. 대기 함수 (토픽으로 좌표 받을 때까지 기다림)
def wait_for_parking_spot(subscriber, timeout=30):
    import time
    print(f"⏳ 주차 위치 수신 대기 중... (최대 {timeout}초)")
    start_time = time.time()
    while time.time() - start_time < timeout:
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        result = subscriber.get_parking_spot()
        if result is not None:
            spot_name, pose = result
            print(f"✅ 주차 위치 수신 완료: {spot_name}")
            return result
        time.sleep(0.1)
    print("⚠️ 주차 위치 수신 실패 (Timeout)")
    return None


# 4. 중앙 제어 노드
class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')

        self.pub_robot0 = self.create_publisher(PoseStamped, '/robot0/nav/goal', 10)
        self.pub_robot2 = self.create_publisher(PoseStamped, '/robot2/nav/goal', 10)
        self.pub_dock_robot0 = self.create_publisher(String, '/robot0/dock_command', 10)
        self.pub_dock_robot2 = self.create_publisher(String, '/robot2/dock_command', 10)

        self.subscriber = ParkingSpotSubscriber()

        self.mqtt_client = connect_mqtt()
        self.mqtt_client.loop_start()

        self.get_logger().info("🚀 중앙 제어 노드 시작됨")

        # MQTT 테스트용 발행 (선택 사항)
        publish(self.mqtt_client, {"id": "0", "location": "B3"})

        # 대기 후 주차 처리 실행
        rclpy.sleep(1.0)  # 초기화 딜레이
        self.handle_parking_logic()

    def handle_parking_logic(self):
        result = wait_for_parking_spot(self.subscriber, timeout=30)
        if result is None:
            self.get_logger().warn("❌ 주차 좌표 수신 실패, 시스템 종료")
            rclpy.shutdown()
            return

        spot_id, pose_msg = result
        self.get_logger().info(f"📦 처리할 주차 ID: {spot_id}")

        if spot_id.startswith("A"):
            self.get_logger().info("🅿️ A구역 → robot2 이동")
            self.pub_robot2.publish(pose_msg)

        elif spot_id.startswith("B"):
            self.get_logger().info(f"🅿️ {spot_id} → robot 간 인계 시나리오 시작")

            # 1. robot2 → waypoint
            wp_pose = self.make_pose(2.5, 2.0)
            self.pub_robot2.publish(wp_pose)
            self.get_logger().info("➡️ robot2 → 인계 지점")

            # 2. 7초 후 robot0 → waypoint
            time.sleep(7)
            self.pub_robot0.publish(wp_pose)
            self.get_logger().info("➡️ robot0 → 인계 지점")

            # 3. robot0 → 목적지
            self.pub_robot0.publish(pose_msg)
            self.get_logger().info(f"➡️ robot0 → {spot_id} 좌표")

            # 4. robot2 도킹
            self.pub_dock_robot2.publish(String(data='dock'))
            self.get_logger().info("➡️ robot2 도킹 명령 전송")

    def make_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    controller = CentralController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("❎ 종료됨")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
