import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YOLOVisualizer(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')

        # YOLOv8 모델 로드 (모델 경로 수정하세요)
        self.model = YOLO('/home/rokey/yolov11/best.pt')

        # 브리지 설정 및 이미지 구독
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/usb_camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("YOLO 시각화 노드가 시작되었습니다.")

    def image_callback(self, msg):
        # ROS 이미지 → OpenCV 이미지
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 추론
        results = self.model(frame)[0]

        # 객체마다 바운딩 박스 표시
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            label = f"{self.model.names[cls_id]} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 0, 255), 2)

        # 디버깅용 이미지 출력
        cv2.imshow('YOLO Detection', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()