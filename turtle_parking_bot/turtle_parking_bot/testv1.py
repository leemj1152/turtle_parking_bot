# yolo_depth_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YOLODepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')
        self.model = YOLO('/home/shin/바탕화면/bacass.pt')
        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None

        self.rgb_sub = self.create_subscription(Image, '/robot2/oakd/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/robot2/oakd/stereo/image_raw', self.depth_callback, 10)
        self.point_pub = self.create_publisher(PointStamped, '/detected_point_cam', 10)

    def correct_depth_linear(self, measured_m):
        return 0.8285 * measured_m + 0.1071

    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.run_detection()

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def run_detection(self):
        if self.rgb_img is None or self.depth_img is None:
            return

        results = self.model(self.rgb_img)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cx_d, cy_d = int(cx / 2) + 80, int(cy / 2) + 80
            if 0 <= cy_d < self.depth_img.shape[0] and 0 <= cx_d < self.depth_img.shape[1]:
                depth_mm = self.depth_img[cy_d, cx_d]
                depth_m = self.correct_depth_linear(depth_mm / 1000.0)

                point = PointStamped()
                point.header.stamp = self.get_clock().now().to_msg()
                point.header.frame_id = 'oakd_rgb_camera_optical_frame'
                point.point.x = 0.0
                point.point.y = 0.0
                point.point.z = depth_m

                self.point_pub.publish(point)
                self.get_logger().info(f"[YOLO] Sent point z={depth_m:.2f} m")

def main():
    rclpy.init()
    node = YOLODepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
