import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_parking_bot.emqx.emqx_pub import connect_mqtt, publish

import ast  # 안전하게 리스트로 파싱

class CenterController(Node):
    def __init__(self):
        super().__init__('center_controller')

        self.subscription = self.create_subscription(
            String,
            '/parking/empty_spot_id',
            self.spot_callback,
            10
        )

        self.mqtt_client = connect_mqtt()
        self.mqtt_client.loop_start()

        self.get_logger().info("CenterController Node started. Listening to /parking/empty_spot_id")

    def spot_callback(self, msg):
        try:
            # 문자열 리스트 → 진짜 리스트로 변환
            zone_list = ast.literal_eval(msg.data)
            if not isinstance(zone_list, list) or len(zone_list) == 0:
                self.get_logger().warn("비어있거나 잘못된 형식의 zone_list 수신됨")
                return

            zone_id = zone_list[0]  # 맨 앞에 있는 zone_id만 사용

            for robot_id in ["0", "2"]:
                payload = {
                    "id": robot_id,
                    "zone_id": zone_id,
                    "type": "start"
                }
                publish(self.mqtt_client, payload)
                self.get_logger().info(f"MQTT 발송 → robot{robot_id}: {payload}")

        except Exception as e:
            self.get_logger().error(f"zone_list 파싱 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CenterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 수신")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
