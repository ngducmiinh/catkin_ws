#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import csv
import rosbag
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates

class SLAMEvaluator:
    def __init__(self, bag_file, slam_method, output_dir=None):
        """
        Khởi tạo bộ đánh giá SLAM
        :param bag_file: Đường dẫn tới file rosbag
        :param slam_method: Phương pháp SLAM (gmapping hoặc hector)
        :param output_dir: Thư mục lưu kết quả
        """
        self.bag_file = bag_file
        self.slam_method = slam_method
        
        # Tạo tên cho file gốc từ tên bag file
        base_name = os.path.basename(bag_file).split('.')[0]
        
        # Thiết lập đường dẫn output
        if output_dir is None:
            output_dir = os.path.expanduser(f"~/catkin_ws/src/custom_robot/evaluation/results/{base_name}")
        
        self.output_dir = output_dir
        
        # Tạo thư mục nếu chưa tồn tại
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # File paths cho output
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.groundtruth_file = os.path.join(self.output_dir, f"groundtruth_{timestamp}.txt")
        self.slam_trajectory_file = os.path.join(self.output_dir, f"{self.slam_method}_trajectory_{timestamp}.txt")
        self.results_file = os.path.join(self.output_dir, f"results_{self.slam_method}_{timestamp}.txt")
    
    def extract_trajectories(self):
        """
        Trích xuất quỹ đạo ground truth và ước lượng từ SLAM từ file rosbag
        """
        print(f"Trích xuất quỹ đạo từ file {self.bag_file}...")
        
        # Mở rosbag để đọc
        bag = rosbag.Bag(self.bag_file)
        
        # Lưu quỹ đạo groundtruth
        groundtruth_data = []
        slam_trajectory_data = []
        
        # Topic cho pose SLAM tùy thuộc vào phương pháp
        slam_pose_topic = f"/{self.slam_method}_slam/pose"
        
        # Đọc dữ liệu từ bag file
        for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states', slam_pose_topic]):
            time_sec = t.to_sec()
            
            if topic == '/gazebo/model_states':
                # Tìm index của mô hình robot trong danh sách các model
                try:
                    robot_idx = msg.name.index('turtlebot3')
                except ValueError:
                    try:
                        # Thử tên khác nếu 'turtlebot3' không có
                        robot_idx = msg.name.index('turtlebot3_burger')
                    except ValueError:
                        continue  # Bỏ qua nếu không tìm thấy robot
                        
                # Lấy pose của robot từ model_states
                pose = msg.pose[robot_idx]
                
                # Chuyển đổi quaternion sang Euler angles
                quaternion = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ]
                roll, pitch, yaw = euler_from_quaternion(quaternion)
                
                # Lưu dữ liệu ground truth theo định dạng TUM: timestamp x y z qx qy qz qw
                # hoặc timestamp x y yaw cho định dạng 2D
                groundtruth_data.append((
                    time_sec,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ))
                
            elif topic == slam_pose_topic:
                # Lấy pose ước lượng từ SLAM
                pose = msg.pose
                
                # Lưu dữ liệu SLAM theo định dạng tương tự
                slam_trajectory_data.append((
                    time_sec,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ))
                
        bag.close()
        
        # Kiểm tra xem có dữ liệu để lưu không
        if not groundtruth_data:
            print("Không tìm thấy dữ liệu ground truth trong bag file!")
            return False
            
        if not slam_trajectory_data:
            print(f"Không tìm thấy dữ liệu quỹ đạo {self.slam_method} trong bag file!")
            return False
        
        # Lưu dữ liệu vào file theo định dạng TUM (timestamp x y z qx qy qz qw)
        with open(self.groundtruth_file, 'w') as f:
            for data in groundtruth_data:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*data))
                
        with open(self.slam_trajectory_file, 'w') as f:
            for data in slam_trajectory_data:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*data))
                
        print(f"Đã trích xuất và lưu {len(groundtruth_data)} điểm ground truth")
        print(f"Đã trích xuất và lưu {len(slam_trajectory_data)} điểm quỹ đạo {self.slam_method}")
        
        return True
    
    def run_evaluation(self):
        """
        Thực hiện đánh giá ATE và RPE sử dụng evo
        """
        print("\nĐánh giá độ chính xác quỹ đạo...")
        
        # Kiểm tra xem evo đã được cài đặt chưa
        try:
            subprocess.run(["evo_ape", "--help"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        except FileNotFoundError:
            print("ERROR: Không tìm thấy evo. Vui lòng cài đặt bằng lệnh: pip install evo")
            print("Tham khảo: https://github.com/MichaelGrupp/evo")
            return False
        
        # Thiết lập các file đầu ra
        ate_plot_file = os.path.join(self.output_dir, f"ate_{self.slam_method}.pdf")
        rpe_plot_file = os.path.join(self.output_dir, f"rpe_{self.slam_method}.pdf")
        
        # Lưu kết quả vào file
        results_file = open(self.results_file, "w")
        
        # Đánh giá ATE (Absolute Trajectory Error)
        print("\n[1/2] Đánh giá ATE (Absolute Trajectory Error)...")
        
        ate_cmd = [
            "evo_ape", "tum", self.groundtruth_file, self.slam_trajectory_file,
            "-p", "--plot_mode", "xy",
            "--save_plot", ate_plot_file,
            "--save_results", os.path.join(self.output_dir, f"ate_{self.slam_method}.zip")
        ]
        
        try:
            ate_result = subprocess.run(ate_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(ate_result.stdout)
            if ate_result.stderr:
                print(f"Lỗi: {ate_result.stderr}")
                
            results_file.write("==== ABSOLUTE TRAJECTORY ERROR (ATE) ====\n")
            results_file.write(ate_result.stdout)
            results_file.write("\n\n")
            
        except Exception as e:
            print(f"Lỗi khi đánh giá ATE: {e}")
            results_file.write(f"Lỗi khi đánh giá ATE: {e}\n")
        
        # Đánh giá RPE (Relative Pose Error)
        print("\n[2/2] Đánh giá RPE (Relative Pose Error)...")
        
        rpe_cmd = [
            "evo_rpe", "tum", self.groundtruth_file, self.slam_trajectory_file,
            "-p", "--plot_mode", "xy",
            "--save_plot", rpe_plot_file,
            "--save_results", os.path.join(self.output_dir, f"rpe_{self.slam_method}.zip")
        ]
        
        try:
            rpe_result = subprocess.run(rpe_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(rpe_result.stdout)
            if rpe_result.stderr:
                print(f"Lỗi: {rpe_result.stderr}")
                
            results_file.write("==== RELATIVE POSE ERROR (RPE) ====\n")
            results_file.write(rpe_result.stdout)
            
        except Exception as e:
            print(f"Lỗi khi đánh giá RPE: {e}")
            results_file.write(f"Lỗi khi đánh giá RPE: {e}\n")
        
        results_file.close()
        
        print(f"\nKết quả đánh giá đã được lưu vào {self.results_file}")
        print(f"Đồ thị ATE: {ate_plot_file}")
        print(f"Đồ thị RPE: {rpe_plot_file}")
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Đánh giá độ chính xác của thuật toán SLAM')
    parser.add_argument('--bag', required=True,
                      help='Đường dẫn tới file rosbag')
    parser.add_argument('--method', required=True, choices=['gmapping', 'hector'],
                      help='Phương pháp SLAM cần đánh giá')
    parser.add_argument('--output_dir', default=None,
                      help='Thư mục lưu kết quả')
    
    args = parser.parse_args()
    
    # Kiểm tra file rosbag có tồn tại không
    if not os.path.isfile(args.bag):
        print(f"Lỗi: File rosbag không tồn tại: {args.bag}")
        return
    
    # Tạo đối tượng đánh giá
    evaluator = SLAMEvaluator(args.bag, args.method, args.output_dir)
    
    # Trích xuất quỹ đạo
    if evaluator.extract_trajectories():
        # Thực hiện đánh giá
        evaluator.run_evaluation()
    else:
        print("Không thể thực hiện đánh giá do lỗi khi trích xuất quỹ đạo.")

if __name__ == "__main__":
    main()