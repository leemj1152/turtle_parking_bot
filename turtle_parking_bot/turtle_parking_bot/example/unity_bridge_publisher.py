import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class UnityBridgePublisher(Node):
    def __init__(self):
        super().__init__('unity_bridge_publisher')

        # rosbridge에 맞는 QoS 설정
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(String, '/chatter', qos)
        self.send_count = 0
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if self.send_count >= 2:
            self.get_logger().info("✅ 2회 전송 완료. 타이머 종료.")
            self.timer.cancel()
            return

        payload = {
            "type": "chatter",
            "msg": {
                "data": f"Hello from ROS2! ({self.send_count + 1})"
            }
        }
        json_str = json.dumps(payload)
        ros_msg = String()
        ros_msg.data = json_str
        self.publisher_.publish(ros_msg)
        self.get_logger().info(f"📤 Published: {json_str}")
        self.send_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = UnityBridgePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
