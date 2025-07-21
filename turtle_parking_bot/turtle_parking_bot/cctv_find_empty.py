import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2

from parking_bot_interfaces.srv import EmptySpots  # ✅ 커스텀 서비스

# === 사용자 정의 ROI ===
# x1=126, y1=189, x2=177, y2=221
# x1=117, y1=234, x2=180, y2=261
# x1=102, y1=263, x2=153, y2=308
A1_ROI = (126, 189, 177, 221)
A2_ROI = (117, 234, 180, 261)
A3_ROI = (102, 263, 153, 308)

# x1=373, y1=315, x2=418, y2=366
# x1=477, y1=321, x2=516, y2=367
# x1=575, y1=318, x2=610, y2=362

B1_ROI = (373, 315, 418, 366)
B2_ROI = (477, 321, 516, 367)
B3_ROI = (575, 318, 610, 362)

PARKING_SPOTS = {
    'A1': A1_ROI,
    'A2': A2_ROI,
    'A3': A3_ROI,
    'B1': B1_ROI,
    'B2': B2_ROI,
    'B3': B3_ROI,
}

YOLO_MODEL_PATH = '/home/rokey/yolov11/best.pt'

def get_overlap_ratio(box, roi):
    """box와 roi의 겹치는 면적이 roi의 몇 %인지 계산"""
    bx1, by1, bx2, by2 = box
    rx1, ry1, rx2, ry2 = roi
    ix1 = max(bx1, rx1)
    iy1 = max(by1, ry1)
    ix2 = min(bx2, rx2)
    iy2 = min(by2, ry2)
    if ix1 >= ix2 or iy1 >= iy2:
        return 0.0
    inter_area = (ix2 - ix1) * (iy2 - iy1)
    roi_area = (rx2 - rx1) * (ry2 - ry1)
    return inter_area / roi_area

class EmptySpotServiceNode(Node):
    def __init__(self):
        super().__init__('empty_spot_service_node')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.latest_frame = None

        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.service = self.create_service(EmptySpots, '/check_empty_spots', self.handle_check_empty_spots)

        self.get_logger().info("✅ 빈자리 서비스 서버가 실행 중입니다.")

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
        results = self.model(frame, conf=0.5, iou=0.5)[0]

        occupied_spots = []

        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            label = f"car {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            for spot_id, roi in PARKING_SPOTS.items():
                if spot_id in occupied_spots:
                    continue
                overlap_ratio = get_overlap_ratio((x1, y1, x2, y2), roi)
                if overlap_ratio >= 0.1:
                    occupied_spots.append(spot_id)
                    self.get_logger().info(f"{spot_id} 점유율: {overlap_ratio:.2f} → 점유로 판단")

        empty_spots = [sid for sid in PARKING_SPOTS if sid not in occupied_spots]
        response.spot_ids = empty_spots

        # ROI 시각화
        for sid, roi in PARKING_SPOTS.items():
            color = (0, 255, 0) if sid in empty_spots else (0, 0, 255)
            x1, y1, x2, y2 = roi
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, sid, (x1 + 3, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 시각화 출력
        cv2.imshow("Empty Spot Detection", frame)
        cv2.waitKey(1)

        self.get_logger().info(f"🚘 빈 주차 구역 응답: {empty_spots}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmptySpotServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⛔ 종료 요청 수신됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


