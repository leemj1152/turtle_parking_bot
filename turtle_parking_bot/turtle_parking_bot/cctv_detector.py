import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

from parking_bot_interfaces.srv import EmptySpots as EmptySpotsSrv  # ✅ 서비스용
from parking_bot_interfaces.msg import EmptySpots                   # ✅ 메시지용

# === 사용자 정의 ===
x1, y1, x2, y2 = 369, 1, 480, 72
ENTRANCE_ROI = (x1, y1, x2, y2)
YOLO_MODEL_PATH = '/home/rokey/yolov11/best.pt'

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)

        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)

        # ✅ 서비스 클라이언트 생성
        self.client = self.create_client(EmptySpotsSrv, '/check_empty_spots')
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('빈자리 확인 서비스가 준비되지 않았습니다. 대기 중...')

        # ✅ 메시지 퍼블리셔 생성
        self.empty_pub = self.create_publisher(EmptySpots, '/parking/empty_spots_msg', 10)

        self.last_entry_time = 0
        self.entry_cooldown = 5.0  # seconds

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        # 입구 ROI 시각화
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.putText(frame, "Entrance ROI", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1b, y1b, x2b, y2b = map(int, box.xyxy[0])
            cx, cy = int((x1b + x2b) / 2), int((y1b + y2b) / 2)

            cv2.rectangle(frame, (x1b, y1b), (x2b, y2b), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)
            cv2.putText(frame, "car", (x1b, y1b - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if ENTRANCE_ROI[0] <= cx <= ENTRANCE_ROI[2] and ENTRANCE_ROI[1] <= cy <= ENTRANCE_ROI[3]:
                if time.time() - self.last_entry_time > self.entry_cooldown:
                    self.get_logger().info("차량 입차 감지 → 빈자리 서비스 요청")
                    self.call_check_empty_service()
                    self.last_entry_time = time.time()
                    cv2.putText(frame, "Entry Detected!", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        cv2.imshow("Entrance Monitor", frame)
        cv2.waitKey(1)

    def call_check_empty_service(self):
        req = EmptySpotsSrv.Request()
        req.trigger = True  # ✅ 요청 트리거

        future = self.client.call_async(req)

        def callback(fut):
            try:
                res = fut.result()
                spot_list = list(res.spot_ids)
                self.get_logger().info(f"응답 받은 빈 자리 목록: {spot_list}")

                # ✅ 퍼블리시
                msg = EmptySpots()
                msg.spot_ids = spot_list
                self.empty_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"서비스 호출 중 오류: {e}")

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
