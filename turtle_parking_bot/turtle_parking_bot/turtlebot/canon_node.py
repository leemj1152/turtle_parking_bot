import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import time

class CanonNode(Node):
    def __init__(self):
        super().__init__('canon_node')
        self.pub = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 10)
        self.sub = self.create_subscription(Bool, '/robot2/audio_stop', self.stop_callback, 10)

        self.should_stop = False
        self.note_index = 0
        self.is_playing = False
        self.next_note_time = 0.0

        # 음계 정의
        self.DO4 = 261.63; self.RE4 = 293.66; self.MI4 = 329.63; self.FA4 = 349.23
        self.SOL4 = 392.00; self.LA4 = 440.00; self.SI4 = 493.88
        self.DO5 = 523.25; self.RE5 = 587.33; self.MI5 = 659.25; self.FA5 = 698.46
        self.SOL5 = 783.99; self.LA5 = 880.00; self.SI5 = 987.77; self.DO6 = 1046.50

        # 캐논 멜로디
        self.canon_melody = [
            (self.SOL5, 0.6), (self.MI5, 0.3), (self.FA5, 0.3), (self.SOL5, 0.6),
            (self.MI5, 0.3), (self.FA5, 0.3), (self.SOL5, 0.3), (self.SI4, 0.3),
            (self.LA4, 0.3), (self.SI4, 0.3), (self.DO5, 0.3), (self.RE5, 0.3),
            (self.MI5, 0.3), (self.FA5, 0.3), (self.MI5, 0.6), (self.DO5, 0.3),
            (self.RE5, 0.3), (self.MI5, 0.6), (self.MI4, 0.3), (self.FA4, 0.3),
            (self.SOL4, 0.3), (self.LA4, 0.3), (self.SOL4, 0.3), (self.FA4, 0.3),
            (self.SOL4, 0.3), (self.DO5, 0.3), (self.SI4, 0.3), (self.DO5, 0.3),
            (self.LA4, 0.6), (self.DO5, 0.3), (self.SI4, 0.3), (self.LA4, 0.6),
            (self.SOL4, 0.3), (self.FA4, 0.3), (self.SOL4, 0.3), (self.FA4, 0.3),
            (self.MI4, 0.3), (self.FA4, 0.3), (self.SOL4, 0.3), (self.LA4, 0.3),
            (self.SI4, 0.3), (self.DO5, 0.3), (self.LA4, 0.6), (self.DO5, 0.3),
            (self.SI4, 0.3), (self.DO5, 0.6), (self.SI4, 0.3), (self.DO5, 0.3),
            (self.SI4, 0.3), (self.LA4, 0.3), (self.SI4, 0.3), (self.DO5, 0.3),
            (self.RE5, 0.3), (self.MI5, 0.3), (self.FA5, 0.3), (self.SOL5, 1.0),
        ]

        # 고정된 타이머 생성 (0.1초마다 체크)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("🚩 종료 신호 수신. 연주 중단합니다.")
            self.should_stop = True
            self.is_playing = False

    def start_continuous_playing(self):
        """연주 시작 - 다른 노드에서 호출할 수 있는 메서드"""
        self.get_logger().info("🎵 연주 시작")
        self.should_stop = False
        self.is_playing = True
        self.note_index = 0
        self.next_note_time = time.time()

    def stop_playing(self):
        """연주 중단 - 다른 노드에서 호출할 수 있는 메서드"""
        self.get_logger().info("🛑 연주 중단")
        self.should_stop = True
        self.is_playing = False

    def timer_callback(self):
        if not self.is_playing or self.should_stop:
            return

        current_time = time.time()
        if current_time >= self.next_note_time:
            self.play_next_note()

    def play_next_note(self):
        if self.note_index >= len(self.canon_melody):
            self.note_index = 0
            self.get_logger().info('🔁 멜로디 반복 시작')

        frequency, duration = self.canon_melody[self.note_index]

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(
                frequency=int(frequency),
                max_runtime=Duration(
                    sec=int(duration),
                    nanosec=int((duration % 1) * 1_000_000_000)
                )
            )
        ]

        note_names = {
            self.DO4: "도4", self.RE4: "레4", self.MI4: "미4", self.FA4: "파4",
            self.SOL4: "솔4", self.LA4: "라4", self.SI4: "시4",
            self.DO5: "도5", self.RE5: "레5", self.MI5: "미5", self.FA5: "파5",
            self.SOL5: "솔5", self.LA5: "라5", self.SI5: "시5", self.DO6: "도6"
        }
        note_name = note_names.get(frequency, f"{frequency:.2f}Hz")

        self.get_logger().info(f'🎼 음표 {self.note_index + 1}: {note_name} ({frequency:.2f}Hz, {duration}초)')

        self.pub.publish(msg)
        self.note_index += 1

        # 다음 노트 재생 시간 설정
        self.next_note_time = time.time() + duration + 0.1


# 독립 실행용 함수
def main(args=None):
    rclpy.init(args=args)
    node = CanonNode()
    
    try:
        node.start_continuous_playing()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트: 연주 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
