
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_parking_bot.emqx.emqx_pub import connect_mqtt, publish
import ast  


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
            zone_list = ast.literal_eval(msg.data)
            if not isinstance(zone_list, list):
                raise ValueError

            def zone_sort_key(zone_id):
                return (zone_id[0], int(zone_id[1:]))
            zone_id = sorted(zone_list, key=zone_sort_key)[0]

            for robot_id in ["0", "2"]:
                payload = {
                    "id": robot_id,
                    "zone_id": zone_id,
                    "type": "start"
                }
                publish(self.mqtt_client, payload)
                self.get_logger().info(f"MQTT 발송 → robot{robot_id}: {payload}")

        except Exception as e:
            self.get_logger().error(f"메시지 파싱 또는 정렬 실패: {e}")


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
