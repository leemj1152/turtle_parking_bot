from parking_bot_interfaces.msg import EmptySpots
import rclpy
from rclpy.node import Node
from turtle_parking_bot.emqx.emqx_pub import EmqxPublisher

class CenterController(Node):
    def __init__(self):
        super().__init__('center_controller')

        self.subscription = self.create_subscription(
            EmptySpots,
            '/parking/empty_spots_msg',
            self.spot_callback,
            10
        )

        self.pub = EmqxPublisher()
        self.pub.start()
        self.get_logger().info("CenterController Node started. Listening to /parking/empty_spots_msg")

def spot_callback(self, msg):
    zone_list = msg.spot_ids  # 빈 자리 리스트

    if not zone_list:
        self.get_logger().warn("수신된 spot_ids 리스트가 비어 있음")
        return

    recommended_spot = recommend_parking_spot(zone_list)
    if not recommended_spot:
        self.get_logger().warn("추천할 수 있는 자리가 없음")
        return

    zone_prefix = recommended_spot[0].upper()
    if zone_prefix == 'A':
        target_robots = ['robot2']
    elif zone_prefix == 'B':
        target_robots = ['robot0', 'robot2']
    else:
        self.get_logger().warn(f"알 수 없는 zone_id: {recommended_spot}")
        return

    for robot_id in target_robots:
        message = {
            "id": robot_id,
            "zone_id": recommended_spot,
            "type": "start"
        }
        self.pub.publish(message)
        self.get_logger().info(f"MQTT 발송 → {robot_id}: {message}")

    def end_node(self):
        self.pub.stop()

# 현재 빈 자리를 리스트로 받아 인접 자리가 없는 자리를 추천
def recommend_parking_spot(empty_spots: list) -> str:
    adjacent_map = {
        'A1': ['A2'],
        'A2': ['A1', 'A3'],
        'A3': ['A2'],
        'B1': ['B2'],
        'B2': ['B1', 'B3'],
        'B3': ['B2'],
    }
    all_spots = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3']
    occupied = [spot for spot in all_spots if spot not in empty_spots]

    for spot in all_spots:
        if spot not in empty_spots:
            continue
        adjacents = adjacent_map.get(spot, []) # 인접한 자리 중 점유된 것이 있는지 확인
        if any(adj in occupied for adj in adjacents):
            continue
        return spot  # 인접 자리에 차량이 없으면 추천

    # 모든 빈자리에 인접 점유 차량이 있어 피한 경우 → 가장 숫자가 낮은 빈 자리 추천
    for spot in all_spots:
        if spot in empty_spots:
            return spot

    return None

def main(args=None):
    rclpy.init(args=args)
    node = CenterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 수신")
    finally:
        node.end_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
