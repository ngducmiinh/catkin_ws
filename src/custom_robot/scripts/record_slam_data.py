#!/usr/bin/env python3

import rospy
import os
import sys
import time
import argparse
import subprocess
from datetime import datetime

def create_directory_if_not_exists(directory):
    """Tạo thư mục nếu nó không tồn tại."""
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            print(f"Đã tạo thư mục: {directory}")
            return True
        except Exception as e:
            print(f"Không thể tạo thư mục {directory}: {e}")
            return False
    return True

def main():
    parser = argparse.ArgumentParser(description='Ghi dữ liệu SLAM để đánh giá')
    parser.add_argument('--method', type=str, default='gmapping',
                      help='Phương pháp SLAM (gmapping hoặc hector)')
    parser.add_argument('--duration', type=int, default=60,
                      help='Thời gian ghi dữ liệu (giây)')
    parser.add_argument('--name', type=str, default=None,
                      help='Tên cho file bag (mặc định: slam_data_method_timestamp)')
    parser.add_argument('--output', type=str, default=None,
                      help='Thư mục lưu file bag (mặc định: ~/catkin_ws/src/custom_robot/evaluation/bags/)')
    parser.add_argument('--environment', type=str, default='real', choices=['sim', 'real'],
                      help='Môi trường: mô phỏng (sim) hoặc robot thật (real)')
    parser.add_argument('--gt_source', type=str, default='odom', 
                      choices=['odom', 'mocap', 'vicon', 'rtabmap', 'manual'],
                      help='Nguồn ground truth cho robot thật (mặc định: odom)')
    args = parser.parse_args()

    if args.method not in ['gmapping', 'hector']:
        print("Lỗi: method phải là 'gmapping' hoặc 'hector'")
        return

    # Tạo tên file dựa trên thời gian
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    env_prefix = "sim" if args.environment == "sim" else "real"
    bag_name = args.name if args.name else f"slam_data_{args.method}_{env_prefix}_{timestamp}"
    
    # Xác định thư mục lưu trữ
    if args.output:
        bag_dir = os.path.expanduser(args.output)
    else:
        # Thử các vị trí lưu trữ khác nhau theo thứ tự ưu tiên
        possible_dirs = [
            os.path.expanduser("~/catkin_ws/src/custom_robot/evaluation/bags/"),
            os.path.expanduser("~/bags/"),
            os.path.expanduser("~/")
        ]
        
        bag_dir = None
        for directory in possible_dirs:
            if create_directory_if_not_exists(directory) and os.access(directory, os.W_OK):
                bag_dir = directory
                break
        
        if bag_dir is None:
            print("Lỗi: Không tìm thấy thư mục nào có quyền ghi.")
            return
    
    # Đảm bảo thư mục tồn tại
    if not create_directory_if_not_exists(bag_dir):
        print(f"Lỗi: Không thể tạo thư mục {bag_dir}")
        return
    
    # Đường dẫn đầy đủ cho file bag
    bag_path = os.path.join(bag_dir, f"{bag_name}.bag")

    print(f"Bắt đầu ghi dữ liệu với phương pháp {args.method} trong {args.duration} giây")
    print(f"Môi trường: {args.environment}")
    if args.environment == 'real':
        print(f"Ground truth source: {args.gt_source}")
    print(f"Dữ liệu sẽ được lưu tại: {bag_path}")

    # Các topic cơ bản cần ghi lại
    topics = [
        "/scan",                           # Dữ liệu LiDAR
        "/tf",                             # Transform frames
        "/tf_static",                      # Static transforms
        "/map",                            # Bản đồ được tạo ra
        "/odom",                           # Odometry từ robot
        "/odom_raw",                       # Dữ liệu odometry thô (nếu có)
        
        # Topic của thuật toán SLAM
        f"/{args.method}_slam/pose",       # Vị trí ước lượng từ thuật toán SLAM
        f"/{args.method}_slam/trajectory", # Quỹ đạo từ thuật toán SLAM (nếu có)
        f"/{args.method}_slam/map"         # Bản đồ từ thuật toán SLAM
    ]
    
    # Thêm topic dựa trên môi trường
    if args.environment == 'sim':
        # Topic chỉ có trong mô phỏng
        topics.append("/gazebo/model_states")  # Ground truth từ mô phỏng (vị trí thật của robot)
    else:
        # Topic cho robot thật
        # Thêm topic dựa trên nguồn ground truth
        if args.gt_source == 'mocap':
            topics.extend(['/mocap/pose', '/mocap/tf', '/mocap/odom'])
        elif args.gt_source == 'vicon':
            topics.extend(['/vicon/pose', '/vicon/tf', '/vicon/odom'])
        elif args.gt_source == 'rtabmap':
            topics.extend(['/rtabmap/odom', '/rtabmap/mapData', '/rtabmap/odomInfo'])
        elif args.gt_source == 'manual':
            topics.extend(['/amcl_pose', '/robot_pose_ekf/odom'])
    
    # Xây dựng chuỗi topic cho rosbag record
    topic_arg = " ".join([f"\"{topic}\"" for topic in topics])
    
    try:
        # Bắt đầu ghi rosbag
        cmd = f"rosbag record -O {bag_path} {topic_arg}"
        print(f"Đang chạy lệnh: {cmd}")
        rosbag_process = subprocess.Popen(cmd, shell=True)
        
        # Chờ đủ thời gian
        print(f"Đang ghi dữ liệu... (sẽ dừng sau {args.duration} giây)")
        time.sleep(args.duration)
        
        # Dừng ghi
        rosbag_process.send_signal(subprocess.signal.SIGINT)
        rosbag_process.wait()
        print(f"Đã ghi xong dữ liệu tại: {bag_path}")
        
        # Hiển thị lệnh để thực hiện đánh giá
        print("\nĐể đánh giá SLAM, hãy chạy lệnh:")
        if args.environment == 'sim':
            print(f"python3 ~/catkin_ws/src/custom_robot/scripts/evaluate_slam.py --bag {bag_path} --method {args.method}")
        else:
            print(f"python3 ~/catkin_ws/src/custom_robot/scripts/evaluate_real_slam.py --bag {bag_path} --method {args.method} --gt_source {args.gt_source}")
        
    except KeyboardInterrupt:
        print("\nĐã dừng ghi dữ liệu")
        rosbag_process.send_signal(subprocess.signal.SIGINT)
        rosbag_process.wait()
        print(f"Dữ liệu đã được lưu tại: {bag_path}")
        
    except Exception as e:
        print(f"Lỗi xảy ra: {e}")

if __name__ == "__main__":
    main()