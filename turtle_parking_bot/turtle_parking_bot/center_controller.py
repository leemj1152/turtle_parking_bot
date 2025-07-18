import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_parking_bot.emqx.emqx_pub import connect_mqtt, publish


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

        self.robot_turn = 0  # 0: robot0, 1: robot2

        self.get_logger().info("CenterController Node started. Listening to /parking/empty_spot_id")

    def spot_callback(self, msg):
        zone_id = msg.data.strip()

        for robot_id in ["0", "2"]:
            payload = {
                "id": robot_id,
                "zone_id": zone_id,
                "type": "start"
            }

            publish(self.mqtt_client, payload) 
            self.get_logger().info(f"MQTT 발송 → robot{robot_id}: {payload}")

        self.robot_turn = 1 - self.robot_turn


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
