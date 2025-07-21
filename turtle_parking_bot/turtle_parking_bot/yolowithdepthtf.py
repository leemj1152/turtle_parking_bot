import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
from ultralytics import YOLO
import time
import threading

MARKER_TOPIC = 'detected_objects_marker'
class YOLOWithDepthTF(Node):
    def __init__(self):
        super().__init__('yolo_with_depth_tf')
        self.model = YOLO('/home/rokey/rokey_ws/install/turtle_parking_bot/share/turtle_parking_bot/model/best.pt')
        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.tf_ready = False
        self.last_infer_time = time.time()
        self.infer_interval = 0.5  # YOLO inference interval (seconds)
        self.rgb_sub = self.create_subscription(CompressedImage, '/robot0/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(ROSImage, '/robot0/oakd/stereo/image_raw', self.depth_callback, 10)
        self.result_pub = self.create_publisher(String, '/robot0/yolo_result', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.enable_tf)
        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0
    def enable_tf(self):
        self.get_logger().info("TF Tree 안정화 완료. TF 변환 활성화.")
        self.tf_ready = True
        self.start_timer.cancel()
    def correct_depth_linear(self, measured_m):
        return 0.8285 * measured_m + 0.1071
    def rgb_callback(self, msg):
        # print('rgb callback 받음')
        self.rgb_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.run_overlay()
    def depth_callback(self, msg):
        # print('depth callback 받음')
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'detected_objects', self.marker_id
        self.marker_id += 1
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        marker.lifetime.sec = 5
        self.marker_pub.publish(marker)
    def run_overlay(self):
        if self.rgb_img is None or self.depth_img is None:
            return
        if time.time() - self.last_infer_time < self.infer_interval:
            return
        print('런 오버레이 실행중')
        self.last_infer_time = time.time()
        rgb_copy = self.rgb_img.copy()

# 코드 수정


        results = self.model(rgb_copy)[0]
        if len(results.boxes) == 0:
            return
        # confidence가 가장 높은 하나만 선택
        best_box = results.boxes[results.boxes.conf.argmax()]
        print('객체 탐지됨')
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        if self.depth_img.shape[:2] != self.rgb_img.shape[:2]:
            scale_x = self.depth_img.shape[1] / self.rgb_img.shape[1]
            scale_y = self.depth_img.shape[0] / self.rgb_img.shape[0]
            cx_d = int(cx * scale_x)
            cy_d = int(cy * scale_y)
        else:
            cx_d, cy_d = int(cx) + 10, int(cy)

        if 0 <= cy_d < self.depth_img.shape[0] and 0 <= cx_d < self.depth_img.shape[1]:
            depth_mm = self.depth_img[cy_d, cx_d]
            depth_m = self.correct_depth_linear(depth_mm / 1000.0)
            depth_text = f"{depth_m:.2f}m"
            print('depth_text', depth_text)
            if self.tf_ready and self.tf_buffer.can_transform('map', 'oakd_rgb_camera_optical_frame', rclpy.time.Time()):
                try:
                    point_cam = PointStamped()
                    point_cam.header.stamp = rclpy.time.Time().to_msg()
                    point_cam.header.frame_id = 'oakd_rgb_camera_optical_frame'
                    point_cam.point.x = 0.0
                    point_cam.point.y = 0.0
                    point_cam.point.z = depth_m
                    point_map = self.tf_buffer.transform(point_cam, 'map')
                    self.get_logger().info(f"[CAM] ({point_cam.point.x:.2f}, {point_cam.point.y:.2f}, {point_cam.point.z:.2f}) → [MAP] ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
                    self.publish_marker(point_map.point.x, point_map.point.y, point_map.point.z,)
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")
        else:
            depth_text = "N/A"
            print('no depth')
        cv2.rectangle(rgb_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(rgb_copy, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(rgb_copy, depth_text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

#코드 수정

        # results = self.model(rgb_copy)[0]
        # for box in results.boxes:
        #     print('객체 탐지됨')
        #     x1, y1, x2, y2 = map(int, box.xyxy[0])
        #     cx = int((x1 + x2) / 2)
        #     cy = int((y1 + y2) / 2)
        #     if self.depth_img.shape[:2] != self.rgb_img.shape[:2]:
        #         scale_x = self.depth_img.shape[1] / self.rgb_img.shape[1]
        #         scale_y = self.depth_img.shape[0] / self.rgb_img.shape[0]
        #         cx_d = int(cx * scale_x)
        #         cy_d = int(cy * scale_y)
        #     else:
        #         cx_d, cy_d = cx + 10, cy
        #     if 0 <= cy_d < self.depth_img.shape[0] and 0 <= cx_d < self.depth_img.shape[1]:
        #         depth_mm = self.depth_img[cy_d, cx_d]
        #         depth_m = self.correct_depth_linear(depth_mm / 1000.0)
        #         depth_text = f"{depth_m:.2f}m"
        #         print('depth_text',depth_text)
        #         if self.tf_ready and self.tf_buffer.can_transform('map', 'oakd_rgb_camera_optical_frame', rclpy.time.Time()):
        #             try:
        #                 point_cam = PointStamped()
        #                 point_cam.header.stamp = rclpy.time.Time().to_msg()
        #                 point_cam.header.frame_id = 'oakd_rgb_camera_optical_frame'
        #                 point_cam.point.x = 0.0
        #                 point_cam.point.y = 0.0
        #                 point_cam.point.z = depth_m
        #                 point_map = self.tf_buffer.transform(point_cam, 'map')
        #                 self.get_logger().info(f"[CAM] ({point_cam.point.x:.2f}, {point_cam.point.y:.2f}, {point_cam.point.z:.2f}) → [MAP] ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
        #                 self.publish_marker(point_map.point.x, point_map.point.y, point_map.point.z,)
        #             except Exception as e:
        #                 self.get_logger().warn(f"TF 변환 실패: {e}")
        #     else:
        #         depth_text = "N/A"
        #         print('no depth')
        #     cv2.rectangle(rgb_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cv2.circle(rgb_copy, (cx, cy), 4, (0, 0, 255), -1)
        #     cv2.putText(rgb_copy, depth_text, (x1, y1 - 10),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            



        depth_vis = np.nan_to_num(self.depth_img, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, 3000)
        depth_vis = (depth_vis / 3000 * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        if depth_color.shape != rgb_copy.shape:
            depth_color = cv2.resize(depth_color, (rgb_copy.shape[1], rgb_copy.shape[0]))
        combined = np.hstack((rgb_copy, depth_color))
        # ros_image = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        msg = String()
        msg.data = depth_text
        self.result_pub.publish(msg)

        cv2.imshow("YOLO + Depth View", combined)
        cv2.waitKey(1)  # 반드시 있어야 OpenCV 윈도우가 갱신됨

def main():
    rclpy.init()
    node = YOLOWithDepthTF()
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        while rclpy.ok():
            time.sleep(0.1)  # sleep to reduce CPU load
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt로 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import numpy as np
# import cv2

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             '/robot0/oakd/rgb/image_raw/compressed',
#             self.image_callback,
#             10)
#         self.get_logger().info('Image subscriber started.')

#     def image_callback(self, msg):
#         # 압축된 이미지 데이터를 numpy 배열로 변환
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         # OpenCV로 디코딩
#         image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         if image is not None:
#             self.get_logger().info(f'Image received: shape={image.shape}')
#             # 이미지 처리 코드 여기에 추가 가능
#             cv2.imshow('Camera Image', image)
#             cv2.waitKey(1)
#         else:
#             self.get_logger().warn('Failed to decode image')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImageSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()