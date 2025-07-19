import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class USBCameraPublisher(Node):
    def __init__(self, cam_index):
        super().__init__('usb_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/usb_camera/image_raw', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f'카메라 {cam_index} 연결 시도 중... 0.5초 대기')
        time.sleep(0.5)

        self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            self.get_logger().error(f'카메라 {cam_index}를 열 수 없습니다.')
            return

        self.get_logger().info(f'카메라 {cam_index} 사용 시작')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"이미지 메시지 생성 또는 퍼블리시 중 오류: {e}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraPublisher(cam_index=0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
