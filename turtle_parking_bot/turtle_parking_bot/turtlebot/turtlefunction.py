import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
from geometry_msgs.msg import PoseStamped, Pose
import emqx.emqx_sub
import time
from std_msgs.msg import Header, String

class TurtleFunction(Node):
    def __init__(self):
        super().__init__('talker')
        self.run_rosbridge()
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        timer_period = 1.0  # 1초마다 메시지 전송
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from ROS2 Python node (VS Code)'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def run_rosbridge(self):
        subprocess.run([
            "ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"
        ])

    def load_pose_from_yaml(yaml_path: str, key: str) -> PoseStamped:
        """YAML 파일에서 PoseStamped 로딩"""
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            pose_data = data[key]['pose']
            header_data = data[key].get('header', {'frame_id': 'map'})
            
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.frame_id = header_data.get('frame_id', 'map')
            
            pose_stamped.pose = Pose()
            pose_stamped.pose.position.x = pose_data['position']['x']
            pose_stamped.pose.position.y = pose_data['position']['y']
            pose_stamped.pose.position.z = pose_data['position'].get('z', 0.0)
            
            pose_stamped.pose.orientation.x = pose_data['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['orientation']['w']
            
            return pose_stamped
        
        except Exception as e:
            print(f"❌ YAML 로딩 실패: {e}")
            raise

    def wait_for_parking_spot(parking_subscriber, timeout=30):
        """주차 위치 토픽 대기"""
        print(f"📡 주차 위치 토픽 대기 중... (최대 {timeout}초)")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(parking_subscriber, timeout_sec=0.1)
            
            parking_spot = parking_subscriber.get_parking_spot()
            if parking_spot is not None:
                print(f"   ✅ 주차 위치 수신: ({parking_spot.pose.position.x:.2f}, {parking_spot.pose.position.y:.2f})")
                return parking_spot
            
            time.sleep(0.1)
        
        print("   ⚠️ 주차 위치 토픽 타임아웃")
        return None
    
    def emqx_run(callback_function):
        emqx.emqx_sub.run(callback_function)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
