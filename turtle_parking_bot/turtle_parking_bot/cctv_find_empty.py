import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

from parking_bot_interfaces.srv import EmptySpots  # ✅ 커스텀 서비스

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

class EmptySpotServiceNode(Node):
    def __init__(self):
        super().__init__('empty_spot_service_node')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.latest_frame = None

        # ✅ 이미지 구독
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)

        # ✅ 서비스 서버 생성
        self.service = self.create_service(EmptySpots, '/check_empty_spots', self.handle_check_empty_spots)

        self.get_logger().info("빈자리 서비스 서버가 실행 중입니다.")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def handle_check_empty_spots(self, request, response):
        if not request.trigger:
            self.get_logger().warn("서비스 요청에서 trigger=False → 처리 생략")
            return response

        if self.latest_frame is None:
            self.get_logger().warn("아직 이미지 프레임 수신 안 됨")
            return response

        frame = self.latest_frame.copy()
        results = self.model(frame)[0]

        occupied_spots = []
        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            for spot_id, roi in PARKING_SPOTS.items():
                if spot_id not in occupied_spots and is_overlap((x1, y1, x2, y2), roi):
                    occupied_spots.append(spot_id)

        empty_spots = [sid for sid in PARKING_SPOTS if sid not in occupied_spots]
        # empty_spots = ['A2']

        response.spot_ids = empty_spots
        self.get_logger().info(f"빈 주차 구역 응답: {empty_spots}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmptySpotServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 수신됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
