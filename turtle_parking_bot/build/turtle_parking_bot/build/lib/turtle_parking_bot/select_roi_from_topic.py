import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROISelector(Node):
    def __init__(self):
        super().__init__('roi_selector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.image_received = False

    def image_callback(self, msg):
        if self.image_received:
            return  # 한 번만 받음

        self.get_logger().info('이미지 수신 완료. ROI 선택을 시작합니다.')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        r = cv2.selectROI("Select ROI", cv_image, fromCenter=False, showCrosshair=True)
        cv2.destroyAllWindows()

        x, y, w, h = map(int, r)
        x1, y1 = x, y
        x2, y2 = x + w, y + h

        print(f"선택된 ROI 좌표: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
        self.image_received = True


def main(args=None):
    rclpy.init(args=args)
    node = ROISelector()

    try:
        while not node.image_received and rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
