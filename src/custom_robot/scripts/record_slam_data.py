#!/usr/bin/env python3

import rospy
import os
import sys
import time
import argparse
import subprocess
from datetime import datetime

def main():
    parser = argparse.ArgumentParser(description='Ghi dữ liệu SLAM để đánh giá')
    parser.add_argument('--method', type=str, default='gmapping',
                      help='Phương pháp SLAM (gmapping hoặc hector)')
    parser.add_argument('--duration', type=int, default=60,
                      help='Thời gian ghi dữ liệu (giây)')
    parser.add_argument('--name', type=str, default=None,
                      help='Tên cho file bag (mặc định: method_timestamp)')
    args = parser.parse_args()

    if args.method not in ['gmapping', 'hector']:
        print("Lỗi: method phải là 'gmapping' hoặc 'hector'")
        return

    # Tạo tên file dựa trên thời gian
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = args.name if args.name else f"{args.method}_{timestamp}"
    bag_path = os.path.expanduser(f"~/catkin_ws/src/custom_robot/evaluation/bags/{bag_name}.bag")

    print(f"Bắt đầu ghi dữ liệu với phương pháp {args.method} trong {args.duration} giây")
    print(f"Dữ liệu sẽ được lưu tại: {bag_path}")

    # Các topic cần ghi lại
    topics = [
        "/scan",                     # Dữ liệu LiDAR
        "/tf",                       # Transform frames
        "/map",                      # Bản đồ được tạo ra
        "/odom",                     # Odometry từ robot
        "/gazebo/model_states",      # Ground truth từ mô phỏng (vị trí thật của robot)
        f"/{args.method}_slam/pose", # Vị trí ước lượng từ thuật toán SLAM
        "/tf_static"                 # Static transforms
    ]

    topic_arg = " ".join([f"-e '{topic}'" for topic in topics])
    
    try:
        # Bắt đầu ghi rosbag
        cmd = f"rosbag record -O {bag_path} {topic_arg}"
        rosbag_process = subprocess.Popen(cmd, shell=True)
        
        # Chờ đủ thời gian
        print(f"Đang ghi dữ liệu... (sẽ dừng sau {args.duration} giây)")
        time.sleep(args.duration)
        
        # Dừng ghi
        rosbag_process.send_signal(subprocess.signal.SIGINT)
        rosbag_process.wait()
        print(f"Đã ghi xong dữ liệu tại: {bag_path}")
        
    except KeyboardInterrupt:
        print("\nĐã dừng ghi dữ liệu")
        rosbag_process.send_signal(subprocess.signal.SIGINT)
        rosbag_process.wait()
        print(f"Dữ liệu đã được lưu tại: {bag_path}")
        
    except Exception as e:
        print(f"Lỗi xảy ra: {e}")

if __name__ == "__main__":
    main()