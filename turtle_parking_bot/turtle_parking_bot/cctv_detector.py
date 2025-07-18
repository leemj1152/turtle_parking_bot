import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

# === 사용자 정의 ROI ===
B1_ROI = (386, 172, 512, 239)
B2_ROI = (488, 124, 609, 203)
B3_ROI = (2, 205, 132, 279)

PARKING_SPOTS = {
    'B1': B1_ROI,
    'B2': B2_ROI,
    'B3': B3_ROI,
}

YOLO_MODEL_PATH = '/home/rokey/yolov11/best.pt'

def is_overlap(box, roi):
    bx1, by1, bx2, by2 = box
    rx1, ry1, rx2, ry2 = roi
    ix1 = max(bx1, rx1)
    iy1 = max(by1, ry1)
    ix2 = min(bx2, rx2)
    iy2 = min(by2, ry2)
    return ix1 < ix2 and iy1 < iy2

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.latest_frame = None

        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.entry_sub = self.create_subscription(Bool, '/parking/entry', self.entry_callback, 10)
        self.spot_pub = self.create_publisher(String, '/parking/empty_spot_id', 10)

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def entry_callback(self, msg):
        if not msg.data or self.latest_frame is None:
            return

        frame = self.latest_frame.copy()
        results = self.model(frame)[0]
        occupied_spots = []

        # ROI 영역 표시
        for spot_id, roi in PARKING_SPOTS.items():
            x1, y1, x2, y2 = roi
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(frame, spot_id, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 차량 감지 표시
        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, 'car', (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            for spot_id, roi in PARKING_SPOTS.items():
                if spot_id not in occupied_spots and is_overlap((x1, y1, x2, y2), roi):
                    self.get_logger().info(f"차량이 {spot_id} 구역에 있습니다.")
                    occupied_spots.append(spot_id)

        empty_spots = [sid for sid in PARKING_SPOTS if sid not in occupied_spots]

        if not empty_spots:
            self.get_logger().info("빈 공간 없음")
        else:
            for spot_id in empty_spots:
                msg = String()
                msg.data = spot_id
                self.get_logger().info(f"빈 자리 발견: {spot_id} → 문자열 전송")
                self.spot_pub.publish(msg)

        # 시각화 결과 출력
        cv2.imshow("Parking Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 수신됨")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
