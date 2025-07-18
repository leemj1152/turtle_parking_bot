#!/usr/bin/env python3

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from geometry_msgs.msg import PoseStamped
import yaml

def load_pose_from_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    pose_data = data['initial_pose']

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = pose_data['header']['frame_id']
    pose_stamped.pose.position.x = pose_data['pose']['position']['x']
    pose_stamped.pose.position.y = pose_data['pose']['position']['y']
    pose_stamped.pose.position.z = pose_data['pose']['position']['z']
    pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
    pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
    pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
    pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']

    return pose_stamped

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    # YAML에서 초기 위치 불러오기
    initial_pose = load_pose_from_yaml('/home/rokey/rokey_ws/maps/tb0_initial_pose.yaml')
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    navigator.undock()

    # 목표 위치 설정 (직접 하드코딩된 목표)
    goal_pose = navigator.getPoseStamped([ -2.00314799838587, 0.8447952841088184], TurtleBot4Directions.NORTH)
    navigator.startToPose(goal_pose)

 

    # 도착 후 도킹
    navigator.dock()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
