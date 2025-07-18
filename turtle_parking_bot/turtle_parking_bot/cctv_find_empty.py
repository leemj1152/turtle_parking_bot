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
    # box와 ROI가 겹치는지 여부
    bx1, by1, bx2, by2 = box
    rx1, ry1, rx2, ry2 = roi

    ix1 = max(bx1, rx1)
    iy1 = max(by1, ry1)
    ix2 = min(bx2, rx2)
    iy2 = min(by2, ry2)

    return ix1 < ix2 and iy1 < iy2  # 겹치면 True

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.latest_frame = None

        # 이미지 구독
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)

        # 입차 감지 신호 구독
        self.entry_sub = self.create_subscription(Bool, '/parking/entry', self.entry_callback, 10)

        # 빈 자리 퍼블리셔
        self.spot_pub = self.create_publisher(String, '/parking/empty_spot_id', 10)

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def entry_callback(self, msg):
        if not msg.data:
            return  # False일 경우 무시

        if self.latest_frame is None:
            self.get_logger().warn("이미지 수신 대기 중")
            return

        results = self.model(self.latest_frame)[0]
        occupied_spots = []
    

        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])

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

if __name__ == '__main__':
    main()
