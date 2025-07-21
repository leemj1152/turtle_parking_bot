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
        zone_list = msg.spot_ids

        if not zone_list:
            self.get_logger().warn("수신된 spot_ids 리스트가 비어 있음")
            return

        zone_id = zone_list[0]  # 첫 번째 zone_id만 사용
        zone_prefix = zone_id[0].upper()

        if zone_prefix == 'A':
            target_robots = ['robot2']
        elif zone_prefix == 'B':
            target_robots = ['robot0', 'robot2']
        else:
            self.get_logger().warn(f"알 수 없는 zone_id: {zone_id}")
            return

        for robot_id in target_robots:
            message = {
                "id": robot_id,
                "zone_id": zone_id,
                "type": "start"
            }
            self.pub.publish(message)
            self.get_logger().info(f"MQTT 발송 → {robot_id}: {message}")

    def end_node(self):
        self.pub.stop()

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
