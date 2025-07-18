#!/usr/bin/env python3
import rclpy
import yaml
import os
import time
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header, String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from rclpy.node import Node

# 요약 : 빈 주차공간을 토픽으로 받아 해당 위치로 터틀봇 이동 후 복귀

# 1. 현재 위치 설정
# 2. undock 실행
# 3. nav2 활성화
# 4. 빈 주차공간 위치로 이동
# 5. pre-dock 위치로 이동
# 6. dock 완료
 
class ParkingSpotSubscriber(Node):
    """주차 위치 토픽 구독자"""
    
    def __init__(self):
        super().__init__('parking_spot_subscriber')
        self.parking_spot = None    # center_node-----------
        self.subscription = self.create_subscription(
            String,
            '/parking/empty_spot_id',
            self.parking_spot_callback,
            10
        )
        self.get_logger().info('주차 위치 토픽 구독 시작: /parking/empty_spot_id')

    def parking_spot_callback(self, msg): # center_node
        spot_name = msg.data.strip()
        coords = get_parking_spot_map_coord(spot_name)  

        if coords is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.orientation.w = 1.0  # 정면 바라보는 기본값
            self.parking_spot = pose
            self.get_logger().info(f'🅿️ 수신한 주차 위치 "{spot_name}" → 좌표: x={coords[0]:.2f}, y={coords[1]:.2f}')
        else:
            self.get_logger().warn(f'❌ 유효하지 않은 주차 위치: "{spot_name}"')  
    
    def get_parking_spot(self):  # center_node---------------------
        """현재 주차 위치 반환"""
        return self.parking_spot

def load_pose_from_yaml(yaml_path: str, key: str) -> PoseStamped:  # robot2
    """YAML 파일에서 PoseStamped 로딩"""
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        pose_data = data[key]['pose']
        header_data = data[key].get('header', {'frame_id': 'map'})
        
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = header_data.get('frame_id', 'map')
        
        pose_stamped.pose = Pose()
        pose_stamped.pose.position.x = pose_data['position']['x']
        pose_stamped.pose.position.y = pose_data['position']['y']
        pose_stamped.pose.position.z = pose_data['position'].get('z', 0.0)
        
        pose_stamped.pose.orientation.x = pose_data['orientation']['x']
        pose_stamped.pose.orientation.y = pose_data['orientation']['y']
        pose_stamped.pose.orientation.z = pose_data['orientation']['z']
        pose_stamped.pose.orientation.w = pose_data['orientation']['w']
        
        return pose_stamped
    
    except Exception as e:
        print(f"❌ YAML 로딩 실패: {e}")
        raise

def get_parking_spot_map_coord(spot_name: str): # center_node
    """
    주차장 빈 칸 이름(예: 'A1')을 받아 맵 좌표(x, y)를 반환.
    없는 칸 이름 입력 시 None 반환
    """
    parking_map = {
        "A1": (0.00814, 0.615),
        "A2": (-1.04, 0.577),
        "A3": (-1.69, 0.528),
        
        "B1": (-2.91, -0.178),
        "B2": (-2.94, -0.569),
        "B3": (-2.96, -1.04)
    }
    
    return parking_map.get(spot_name, None)

def wait_for_parking_spot(parking_subscriber, timeout=30):
    """주차 위치 토픽 대기"""
    print(f"📡 주차 위치 토픽 대기 중... (최대 {timeout}초)")
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        rclpy.spin_once(parking_subscriber, timeout_sec=0.1)
        
        parking_spot = parking_subscriber.get_parking_spot()
        if parking_spot is not None:
            print(f"   ✅ 주차 위치 수신: ({parking_spot.pose.position.x:.2f}, {parking_spot.pose.position.y:.2f})")
            return parking_spot
        
        time.sleep(0.1)
    
    print("   ⚠️ 주차 위치 토픽 타임아웃")
    return None

def main():
    print("🚀 TurtleBot4 Navigation 시작 (주차 및 도킹 포함)")
    
    # ROS2 초기화
    rclpy.init()
    
    # 네임스페이스를 /robot2로 설정
    navigator = TurtleBot4Navigator(namespace='/robot2')
    
    # 주차 위치 구독자 생성
    parking_subscriber = ParkingSpotSubscriber()
    
    try:
        # 1. 초기 위치 설정 (undocking 전에 수행)
        print("1️⃣ 초기 위치 설정 중...")
        try:
            initial_path = os.path.expanduser('~/rokey_ws/maps/tb2_initial_pose.yaml')
            initial_pose = load_pose_from_yaml(initial_path, 'initial_pose')
            # frame_id도 네임스페이스에 맞게 설정
            initial_pose.header.frame_id = 'map'
            navigator.setInitialPose(initial_pose)
            print(f"   ✅ 초기 위치 설정: ({initial_pose.pose.position.x:.2f}, {initial_pose.pose.position.y:.2f})")
        except Exception as e:
            print(f"   ⚠️ 초기 위치 설정 실패: {e}")
            print("   → 기본 위치로 계속 진행...")
        
        # 2. Undocking 수행
        print("2️⃣ Undocking 수행 중...")
        try:
            navigator.undock()
            print("   ✅ Undocking 완료")
        except Exception as e:
            print(f"   ⚠️ Undocking 실패: {e}")
            print("   → 이미 undock 상태일 수 있습니다.")
        
        # 3. Nav2 활성화 대기
        print("3️⃣ Nav2 활성화 대기 중...")
        print("   → 이 과정은 시간이 걸릴 수 있습니다...")
        
        # 타임아웃 설정 (60초로 늘림)
        import threading
        nav2_ready = threading.Event()
        
        def wait_for_nav2():
            try:
                navigator.waitUntilNav2Active()
                nav2_ready.set()
            except Exception as e:
                print(f"   ❌ Nav2 활성화 실패: {e}")
        
        nav2_thread = threading.Thread(target=wait_for_nav2)
        nav2_thread.start()
        
        if nav2_ready.wait(timeout=60):
            print("   ✅ Nav2 활성화 완료")
        else:
            print("   ⚠️ Nav2 활성화 타임아웃 (60초)")
            print("   → Nav2가 실행되지 않았을 수 있습니다.")
            
            # 서비스 상태 확인
            print("   → 서비스 상태 확인 중...")
            import subprocess
            try:
                result = subprocess.run(['ros2', 'service', 'list', '|', 'grep', 'robot2'], 
                                      shell=True, capture_output=True, text=True)
                print(f"   → /robot2 서비스들: {result.stdout}")
            except:
                pass
            return
        
        # 4. 주차 위치 토픽 대기
        print("4️⃣ 주차 위치 토픽 대기 중...")
        parking_spot = wait_for_parking_spot(parking_subscriber, timeout=30)
        
        if parking_spot is None:
            print("   ❌ 주차 위치를 받을 수 없습니다. 프로그램을 종료합니다.")
            return
        
        # 5. 주차 위치로 이동
        print("5️⃣ 주차 위치로 이동 중...")
        # PoseStamped를 그대로 사용
        parking_pose = parking_spot
        parking_pose.header.frame_id = 'map'  # frame_id 확인/설정
        print(f"   → 주차 위치: ({parking_spot.pose.position.x:.2f}, {parking_spot.pose.position.y:.2f})")
        
        navigator.goToPose(parking_pose)
        
        # 네비게이션 완료 대기 (타임아웃 추가)
        timeout = 60  # 60초 타임아웃
        elapsed = 0
        while not navigator.isTaskComplete() and elapsed < timeout:
            time.sleep(0.1)
            elapsed += 0.1
            
        if elapsed >= timeout:
            print("   ⚠️ 주차 위치 도착 타임아웃")
        else:
            print("   ✅ 주차 위치 도착 완료")
        
        # 6. Pre-dock 위치로 이동
        print("6️⃣ Pre-dock 위치로 이동 중...")
        try:
            predock_path = os.path.expanduser('~/rokey_ws/maps/tb2_pre_dock_pose.yaml')
            pre_dock_pose = load_pose_from_yaml(predock_path, 'pre_dock_pose')

            print(f"   → Pre-dock: ({pre_dock_pose.pose.position.x:.2f}, {pre_dock_pose.pose.position.y:.2f})")
            
            navigator.goToPose(pre_dock_pose)
            
            # 네비게이션 완료 대기 (타임아웃 추가)
            timeout = 60  # 60초 타임아웃
            elapsed = 0
            while not navigator.isTaskComplete() and elapsed < timeout:
                time.sleep(0.1)
                elapsed += 0.1
                
            if elapsed >= timeout:
                print("   ⚠️ Pre-dock 위치 도착 타임아웃")
            else:
                print("   ✅ Pre-dock 위치 도착 완료")
                
        except Exception as e:
            print(f"   ⚠️ Pre-dock 이동 실패: {e}")
        
        # 7. 도킹 수행
        print("7️⃣ 도킹 수행 중...")
        try:
            navigator.dock()
            print("   ✅ 도킹 완료")
        except Exception as e:
            print(f"   ❌ 도킹 실패: {e}")
        
        print("✅ 네비게이션 완료 (주차 및 도킹 포함)")
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        rclpy.shutdown()
        print("🏁 프로그램 종료")

if __name__ == '__main__':
    main()