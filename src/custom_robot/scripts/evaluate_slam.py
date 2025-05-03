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
from tf.transformations import euler_from_quaternion, quaternion_matrix
from geometry_msgs.msg import PoseStamped, Pose, Transform
from gazebo_msgs.msg import ModelStates
import tf
import tf2_py
import tf2_ros
import rospy
from collections import defaultdict
import time

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
    
    def extract_transform_data(self, trans, rot):
        """
        Trích xuất dữ liệu vị trí và góc quay từ transform
        :param trans: Translation (x, y, z)
        :param rot: Rotation quaternion (x, y, z, w)
        :return: [x, y, z, qx, qy, qz, qw]
        """
        return [
            trans[0], trans[1], trans[2],
            rot[0], rot[1], rot[2], rot[3]
        ]
        
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
        
        # Các biến để xử lý TF
        tf_buffer = {}  # Lưu trữ các transform
        map_to_odom_transforms = {}  # Lưu các transform từ map đến odom
        odom_to_base_transforms = {}  # Lưu các transform từ odom đến base
        
        print("Quét các bản tin tf từ rosbag...")
        
        # Đọc tất cả các bản tin tf và lưu trữ
        for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
            time_sec = t.to_sec()
            
            for transform in msg.transforms:
                key = (transform.header.frame_id, transform.child_frame_id)
                value = (transform.transform, time_sec)
                
                if key not in tf_buffer:
                    tf_buffer[key] = []
                tf_buffer[key].append(value)
                
                # Lưu các transform quan trọng
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    map_to_odom_transforms[time_sec] = transform.transform
                elif transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                    odom_to_base_transforms[time_sec] = transform.transform
        
        # Mảng các timestamp để xử lý theo thời gian
        all_times = sorted(set(list(map_to_odom_transforms.keys()) + list(odom_to_base_transforms.keys())))
        
        print(f"Đã tìm thấy {len(map_to_odom_transforms)} transform map->odom và {len(odom_to_base_transforms)} transform odom->base_footprint")
        
        # Đọc dữ liệu từ bag file để lấy ground truth từ gazebo/model_states
        print("Đọc dữ liệu ground truth từ Gazebo...")
        for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
            time_sec = t.to_sec()
            
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
            
            # Lưu dữ liệu ground truth theo định dạng TUM: timestamp x y z qx qy qz qw
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
        
        # Cố gắng đọc từ topic pose trước
        has_pose_topic = False
        print(f"Thử đọc dữ liệu từ topic pose của SLAM: {slam_pose_topic}...")
        for topic, msg, t in bag.read_messages(topics=[slam_pose_topic]):
            has_pose_topic = True
            time_sec = t.to_sec()
            
            # Lấy pose ước lượng từ SLAM
            pose = msg.pose
            
            # Lưu dữ liệu SLAM
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
        
        # Nếu không tìm thấy topic pose, sử dụng TF để tính quỹ đạo
        if not has_pose_topic:
            print(f"Không tìm thấy topic pose của {self.slam_method}, sử dụng transform (tf) để tính quỹ đạo...")
            
            # Đảm bảo chúng ta có đủ transform để tính quỹ đạo
            if len(map_to_odom_transforms) == 0 or len(odom_to_base_transforms) == 0:
                print("Không đủ dữ liệu transform để tính quỹ đạo SLAM!")
                bag.close()
                return False
                
            # Những thời điểm của odom để sử dụng
            odom_times = sorted(odom_to_base_transforms.keys())
            
            # Với mỗi transform odom->base, tìm transform map->odom gần nhất về thời gian
            # và kết hợp để có transform map->base (quỹ đạo SLAM)
            for odom_time in odom_times:
                # Tìm transform map->odom gần nhất với thời điểm hiện tại
                closest_map_time = min(map_to_odom_transforms.keys(), 
                                     key=lambda x: abs(x - odom_time)) if map_to_odom_transforms else None
                
                if closest_map_time is None:
                    continue
                    
                # Lấy các transform
                map_to_odom = map_to_odom_transforms[closest_map_time]
                odom_to_base = odom_to_base_transforms[odom_time]
                
                # Tính transform kết hợp từ map đến base
                # Tạo ma trận chuyển đổi từ map đến odom
                mo_trans = [map_to_odom.translation.x, map_to_odom.translation.y, map_to_odom.translation.z]
                mo_quat = [map_to_odom.rotation.x, map_to_odom.rotation.y, 
                          map_to_odom.rotation.z, map_to_odom.rotation.w]
                mo_matrix = tf.transformations.quaternion_matrix(mo_quat)
                mo_matrix[0:3, 3] = mo_trans
                
                # Tạo ma trận chuyển đổi từ odom đến base
                ob_trans = [odom_to_base.translation.x, odom_to_base.translation.y, odom_to_base.translation.z]
                ob_quat = [odom_to_base.rotation.x, odom_to_base.rotation.y, 
                          odom_to_base.rotation.z, odom_to_base.rotation.w]
                ob_matrix = tf.transformations.quaternion_matrix(ob_quat)
                ob_matrix[0:3, 3] = ob_trans
                
                # Kết hợp hai ma trận chuyển đổi
                mb_matrix = np.dot(mo_matrix, ob_matrix)
                
                # Trích xuất thông tin vị trí và góc quay từ ma trận kết hợp
                mb_trans = mb_matrix[0:3, 3]
                mb_quat = tf.transformations.quaternion_from_matrix(mb_matrix)
                
                # Lưu thông tin quỹ đạo SLAM
                slam_trajectory_data.append((
                    odom_time,  # Thời gian
                    mb_trans[0], mb_trans[1], mb_trans[2],  # Vị trí
                    mb_quat[0], mb_quat[1], mb_quat[2], mb_quat[3]  # Góc quay
                ))
        
        bag.close()
        
        # Kiểm tra xem có dữ liệu để lưu không
        if not groundtruth_data:
            print("Không tìm thấy dữ liệu ground truth trong bag file!")
            return False
            
        if not slam_trajectory_data:
            print(f"Không tìm thấy dữ liệu quỹ đạo {self.slam_method} trong bag file!")
            return False
        
        # Sắp xếp dữ liệu theo thời gian
        groundtruth_data.sort(key=lambda x: x[0])
        slam_trajectory_data.sort(key=lambda x: x[0])
        
        print(f"Đã trích xuất {len(groundtruth_data)} điểm dữ liệu ground truth")
        print(f"Đã trích xuất {len(slam_trajectory_data)} điểm dữ liệu quỹ đạo {self.slam_method}")
        
        # Lưu dữ liệu vào file theo định dạng TUM (timestamp x y z qx qy qz qw)
        with open(self.groundtruth_file, 'w') as f:
            for data in groundtruth_data:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*data))
                
        with open(self.slam_trajectory_file, 'w') as f:
            for data in slam_trajectory_data:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*data))
                
        print(f"Đã lưu dữ liệu ground truth vào: {self.groundtruth_file}")
        print(f"Đã lưu dữ liệu quỹ đạo {self.slam_method} vào: {self.slam_trajectory_file}")
        
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
            "--save_results", os.path.join(self.output_dir, f"ate_{self.slam_method}.zip"),
            "--align", "--correct_scale"  # Thêm các tùy chọn căn chỉnh và hiệu chỉnh tỷ lệ
        ]
        
        try:
            ate_result = subprocess.run(ate_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(ate_result.stdout)
            if ate_result.stderr and "error" in ate_result.stderr.lower():
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
            "--save_results", os.path.join(self.output_dir, f"rpe_{self.slam_method}.zip"),
            "--align", "--correct_scale"  # Thêm các tùy chọn căn chỉnh và hiệu chỉnh tỷ lệ
        ]
        
        try:
            rpe_result = subprocess.run(rpe_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(rpe_result.stdout)
            if rpe_result.stderr and "error" in rpe_result.stderr.lower():
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