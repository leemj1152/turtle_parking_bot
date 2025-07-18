# from parking_bot_interfaces.msg import EmptySpots  # ✅ 메시지 타입
# import time
# import rclpy
# from rclpy.node import Node
# import turtle_parking_bot.emqx.emqx_pub as emqx_pub
# from emqx.emqx_pub import EmqxPublisher
# import time

# class CenterController(Node):
#     def __init__(self):
#         super().__init__('center_controller')

#         self.subscription = self.create_subscription(
#             EmptySpots,                             # ✅ 메시지 타입 수정
#             '/parking/empty_spots_msg',               # ✅ 해당 토픽에서 메시지 수신
#             self.spot_callback,
#             10
#         )
#         self.pub = EmqxPublisher()
#         self.pub.start()
#         self.get_logger().info("CenterController Node started. Listening to /parking/empty_spot_id")

#     def spot_callback(self, msg):
#         zone_list = msg.spot_ids  # ✅ 이제는 파싱 없이 바로 리스트

#         if not zone_list:
#             self.get_logger().warn("수신된 spot_ids 리스트가 비어 있음")
#             return

#         zone_id = zone_list[0]  # 맨 앞 요소 사용

#         message = {
#                 "id": 'all',
#                 "zone_id": zone_id,
#                 "type": "start"
#         }
#         self.pub.publish(message)
#             # time.sleep(2)
#             # self.client.loop_stop()
#             # publish(self.mqtt_client, payload)
#             # self.get_logger().info(f"MQTT 발송 → robot{robot_id}: {payload}")
#     def end_node(self):
#         self.pub.stop() 

# def main(args=None):
#     rclpy.init(args=args)
#     node = CenterController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("종료 요청 수신")
#     finally:
#         node.end_node()
#         node.destroy_node()
#         rclpy.shutdown()
        

# if __name__ == '__main__':
#     main()


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
            target_robots = ['robot0', 'robot1']
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
