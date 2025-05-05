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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Transform, TransformStamped
import tf
import tf2_py
import tf2_ros
import rospy
from collections import defaultdict
import time
import yaml
import math
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu

class RealRobotSLAMEvaluator:
    def __init__(self, bag_file, slam_method, gt_source='mocap', output_dir=None):
        """
        Khởi tạo bộ đánh giá SLAM cho robot thật
        :param bag_file: Đường dẫn tới file rosbag
        :param slam_method: Phương pháp SLAM (gmapping hoặc hector)
        :param gt_source: Nguồn dữ liệu ground truth ('mocap', 'vicon', 'rtabmap', 'manual')
        :param output_dir: Thư mục lưu kết quả
        """
        self.bag_file = bag_file
        self.slam_method = slam_method
        self.gt_source = gt_source
        
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
        
        # Topics dựa trên nguồn ground truth
        self.gt_topics = self._get_gt_topics()
        self.slam_topics = self._get_slam_topics()
        
    def _get_gt_topics(self):
        """
        Trả về danh sách các topics chứa thông tin ground truth dựa vào nguồn
        """
        if self.gt_source == 'mocap':
            return ['/mocap/pose', '/mocap/tf', '/mocap/odom']
        elif self.gt_source == 'vicon':
            return ['/vicon/pose', '/vicon/tf', '/vicon/odom']
        elif self.gt_source == 'rtabmap':
            return ['/rtabmap/odom', '/rtabmap/mapData', '/rtabmap/odomInfo']
        elif self.gt_source == 'manual':
            # Sử dụng bản đồ tự chuẩn bị và một thuật toán định vị chính xác
            return ['/amcl_pose', '/robot_pose_ekf/odom']
        else:
            # Mặc định, sử dụng odom làm GT (không lý tưởng nhưng có thể dùng làm baseline)
            return ['/odom', '/tf']
            
    def _get_slam_topics(self):
        """
        Trả về danh sách các topics chứa thông tin quỹ đạo ước lượng từ SLAM
        """
        topics = [f'/{self.slam_method}_slam/pose', f'/{self.slam_method}_slam/trajectory', 
                  f'/{self.slam_method}_slam/map', '/tf', '/tf_static']
        return topics
        
    def extract_trajectories(self):
        """
        Trích xuất quỹ đạo ground truth và ước lượng từ SLAM từ file rosbag
        """
        print(f"Trích xuất quỹ đạo từ file {self.bag_file}...")
        
        # Mở rosbag để đọc
        bag = rosbag.Bag(self.bag_file)
        
        # Lưu quỹ đạo groundtruth và slam
        groundtruth_data = []
        slam_trajectory_data = []
        
        # Thiết lập các chủ đề theo phương pháp SLAM
        slam_pose_topic = f"/{self.slam_method}_slam/pose"
        
        # Các biến để xử lý TF
        tf_buffer = {}  # Lưu trữ các transform
        map_to_odom_transforms = {}  # Lưu các transform từ map đến odom
        odom_to_base_transforms = {}  # Lưu các transform từ odom đến base
        gt_transforms = {}  # Lưu các transform ground truth
        
        print("Quét các bản tin tf từ rosbag...")
        
        # Xác định frame IDs dựa trên nguồn ground truth
        if self.gt_source == 'mocap':
            gt_frame = 'mocap'
        elif self.gt_source == 'vicon':
            gt_frame = 'vicon'
        elif self.gt_source == 'rtabmap':
            gt_frame = 'rtabmap'
        else:
            gt_frame = 'odom'  # Mặc định
        
        base_frame = 'base_link'  # hoặc 'base_footprint' tùy theo robot
        
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
                elif transform.header.frame_id == 'odom' and (transform.child_frame_id == 'base_link' or transform.child_frame_id == 'base_footprint'):
                    odom_to_base_transforms[time_sec] = transform.transform
                    base_frame = transform.child_frame_id  # Cập nhật frame chính xác của robot
                
                # Lưu transform ground truth
                if gt_frame in transform.header.frame_id and (transform.child_frame_id == 'base_link' or transform.child_frame_id == 'base_footprint'):
                    gt_transforms[time_sec] = transform.transform
        
        print(f"Đã tìm thấy {len(map_to_odom_transforms)} transform map->odom và {len(odom_to_base_transforms)} transform odom->base")
        print(f"Đã tìm thấy {len(gt_transforms)} transform từ {gt_frame} đến {base_frame}")
        
        # Trích xuất ground truth dựa vào nguồn
        gt_extracted = False
        
        # 1. Thử đọc từ topic pose/odom ground truth trước
        gt_pose_topics = []
        if self.gt_source == 'mocap':
            gt_pose_topics = ['/mocap/pose', '/mocap/odom']
        elif self.gt_source == 'vicon':
            gt_pose_topics = ['/vicon/pose', '/vicon/odom']
        elif self.gt_source == 'rtabmap':
            gt_pose_topics = ['/rtabmap/odom']
        elif self.gt_source == 'manual':
            gt_pose_topics = ['/amcl_pose', '/robot_pose_ekf/odom']
        else:
            gt_pose_topics = ['/odom']
        
        print(f"Thử đọc ground truth từ các topic: {gt_pose_topics}")
        
        for topic, msg, t in bag.read_messages(topics=gt_pose_topics):
            time_sec = t.to_sec()
            
            # Trích xuất pose từ message
            if topic.endswith('/pose') or 'amcl' in topic:
                if hasattr(msg, 'pose'):
                    pose = msg.pose
                else:
                    pose = msg
                
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
                gt_extracted = True
            elif topic.endswith('/odom'):
                pose = msg.pose.pose
                
                # Lưu dữ liệu ground truth
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
                gt_extracted = True
        
        # 2. Nếu không tìm thấy dữ liệu từ topic, sử dụng TF
        if not gt_extracted and gt_transforms:
            print(f"Không tìm thấy dữ liệu ground truth từ topics, sử dụng transform từ {gt_frame}")
            
            for time_sec, transform in gt_transforms.items():
                # Trích xuất vị trí và góc quay
                trans = [transform.translation.x, transform.translation.y, transform.translation.z]
                rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
                
                # Lưu dữ liệu ground truth
                groundtruth_data.append((
                    time_sec,
                    trans[0], trans[1], trans[2],
                    rot[0], rot[1], rot[2], rot[3]
                ))
                gt_extracted = True
        
        # 3. Nếu không có ground truth thực tế, tạo ground truth giả lập
        if not gt_extracted:
            print("Không tìm thấy dữ liệu ground truth thực tế, tạo ground truth từ odometry")
            
            # Sử dụng odometry làm ground truth
            for topic, msg, t in bag.read_messages(topics=['/odom']):
                time_sec = t.to_sec()
                pose = msg.pose.pose
                
                # Lưu dữ liệu odometry làm ground truth
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
                gt_extracted = True
        
        # Trích xuất quỹ đạo SLAM
        slam_extracted = False
        
        # 1. Thử đọc từ topic pose/trajectory của SLAM trước
        print(f"Thử đọc quỹ đạo SLAM từ topic: {slam_pose_topic}")
        for topic, msg, t in bag.read_messages(topics=[slam_pose_topic, f'/{self.slam_method}_slam/trajectory']):
            slam_extracted = True
            time_sec = t.to_sec()
            
            # Trích xuất pose
            if isinstance(msg, PoseStamped):
                pose = msg.pose
            elif hasattr(msg, 'poses') and len(msg.poses) > 0:  # Path message
                pose = msg.poses[-1].pose
            else:
                pose = msg
            
            # Lưu dữ liệu quỹ đạo SLAM
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
        
        # 2. Nếu không tìm thấy topic, sử dụng TF để tính quỹ đạo
        if not slam_extracted:
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
                slam_extracted = True
        
        bag.close()
        
        # Kiểm tra xem có dữ liệu để lưu không
        if not gt_extracted or len(groundtruth_data) == 0:
            print("Không tìm thấy dữ liệu ground truth trong bag file!")
            return False
            
        if not slam_extracted or len(slam_trajectory_data) == 0:
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

    def create_synthetic_ground_truth(self):
        """
        Tạo ground truth tổng hợp khi không có nguồn ground truth thực tế
        Có thể tích hợp hai thuật toán SLAM để tạo ground truth tốt hơn
        """
        print("Tạo ground truth tổng hợp từ nhiều nguồn...")
        
        # Phương pháp này có thể được thực hiện bằng cách:
        # 1. Chạy thêm một thuật toán SLAM nữa (RTABMap, Cartographer) với cấu hình chất lượng cao
        # 2. Sử dụng kết hợp của odometry và IMU để cải thiện ước lượng quỹ đạo
        # 3. Nếu có dữ liệu từ nhiều lần chạy, có thể kết hợp để tạo map tốt hơn
        
        try:
            # Đây chỉ là placeholder cho việc tạo ground truth tổng hợp
            # Trong thực tế, phương pháp này đòi hỏi nhiều xử lý phức tạp hơn
            subprocess.run([
                "roslaunch", "custom_robot", "rtabmap_ground_truth.launch", 
                f"bag_file:={self.bag_file}"
            ], check=True)
            
            # Đường dẫn tới file ground truth do RTABMap tạo ra
            rtabmap_gt_file = os.path.join(os.path.dirname(self.bag_file), "rtabmap_trajectory.txt")
            
            # Nếu file tồn tại, sử dụng làm ground truth
            if os.path.exists(rtabmap_gt_file):
                import shutil
                shutil.copy(rtabmap_gt_file, self.groundtruth_file)
                return True
                
        except Exception as e:
            print(f"Không thể tạo ground truth tổng hợp: {e}")
        
        return False
    
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
        
        # Tạo file so sánh nếu đã chạy cả hai thuật toán
        gmapping_file = os.path.join(os.path.dirname(self.results_file), "results_gmapping_")
        hector_file = os.path.join(os.path.dirname(self.results_file), "results_hector_")
        
        gmapping_files = [f for f in os.listdir(os.path.dirname(self.results_file)) if f.startswith("results_gmapping_")]
        hector_files = [f for f in os.listdir(os.path.dirname(self.results_file)) if f.startswith("results_hector_")]
        
        if gmapping_files and hector_files:
            self.compare_results(
                os.path.join(os.path.dirname(self.results_file), gmapping_files[-1]),
                os.path.join(os.path.dirname(self.results_file), hector_files[-1])
            )
        
        print(f"\nKết quả đánh giá đã được lưu vào {self.results_file}")
        print(f"Đồ thị ATE: {ate_plot_file}")
        print(f"Đồ thị RPE: {rpe_plot_file}")
        
        return True
    
    def compare_results(self, gmapping_results, hector_results):
        """
        So sánh kết quả giữa gmapping và hector
        """
        print("\nSo sánh kết quả giữa gmapping và hector...")
        
        # Đọc kết quả từ file
        gmapping_ate = self._extract_metric(gmapping_results, "ATE")
        gmapping_rpe = self._extract_metric(gmapping_results, "RPE")
        hector_ate = self._extract_metric(hector_results, "ATE")
        hector_rpe = self._extract_metric(hector_results, "RPE")
        
        comparison_file = os.path.join(self.output_dir, "comparison.txt")
        
        with open(comparison_file, 'w') as f:
            f.write("==== SO SÁNH GMAPPING VÀ HECTOR SLAM ====\n\n")
            
            f.write("=== ABSOLUTE TRAJECTORY ERROR (ATE) ===\n")
            f.write(f"GMMapping ATE RMSE: {gmapping_ate['rmse']:.6f} m\n")
            f.write(f"Hector ATE RMSE: {hector_ate['rmse']:.6f} m\n")
            f.write(f"Chênh lệch: {abs(gmapping_ate['rmse'] - hector_ate['rmse']):.6f} m\n")
            if gmapping_ate['rmse'] < hector_ate['rmse']:
                f.write("GMMapping có sai số thấp hơn trong ATE.\n\n")
            else:
                f.write("Hector có sai số thấp hơn trong ATE.\n\n")
                
            f.write("=== RELATIVE POSE ERROR (RPE) ===\n")
            f.write(f"GMMapping RPE RMSE: {gmapping_rpe['rmse']:.6f} m/m\n")
            f.write(f"Hector RPE RMSE: {hector_rpe['rmse']:.6f} m/m\n")
            f.write(f"Chênh lệch: {abs(gmapping_rpe['rmse'] - hector_rpe['rmse']):.6f} m/m\n")
            if gmapping_rpe['rmse'] < hector_rpe['rmse']:
                f.write("GMMapping có sai số thấp hơn trong RPE.\n\n")
            else:
                f.write("Hector có sai số thấp hơn trong RPE.\n\n")
            
            f.write("=== ĐÁNH GIÁ TỔNG QUAN ===\n")
            if gmapping_ate['rmse'] + gmapping_rpe['rmse'] < hector_ate['rmse'] + hector_rpe['rmse']:
                f.write("GMMapping tổng thể có độ chính xác cao hơn trong cả ATE và RPE.\n")
            else:
                f.write("Hector tổng thể có độ chính xác cao hơn trong cả ATE và RPE.\n")
            
        print(f"Đã lưu kết quả so sánh vào {comparison_file}")
        
        # Tạo biểu đồ so sánh
        try:
            self._plot_comparison(gmapping_ate, hector_ate, gmapping_rpe, hector_rpe)
        except Exception as e:
            print(f"Không thể tạo biểu đồ so sánh: {e}")
        
        return True
    
    def _extract_metric(self, result_file, metric_type):
        """
        Trích xuất chỉ số từ file kết quả
        """
        metrics = {'rmse': 0.0, 'mean': 0.0, 'median': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0}
        
        try:
            with open(result_file, 'r') as f:
                content = f.read()
                
            # Tìm phần chứa metric cần trích xuất
            if metric_type == "ATE":
                section = content.split("==== ABSOLUTE TRAJECTORY ERROR")[1].split("====")[0]
            elif metric_type == "RPE":
                section = content.split("==== RELATIVE POSE ERROR")[1]
            else:
                return metrics
            
            # Trích xuất chỉ số RMSE
            for line in section.split('\n'):
                if "rmse" in line.lower():
                    metrics['rmse'] = float(line.split(':')[1].strip().split()[0])
                elif "mean" in line.lower() and "mean error" not in line.lower():
                    metrics['mean'] = float(line.split(':')[1].strip().split()[0])
                elif "median" in line.lower():
                    metrics['median'] = float(line.split(':')[1].strip().split()[0])
                elif "std" in line.lower():
                    metrics['std'] = float(line.split(':')[1].strip().split()[0])
                elif "min" in line.lower():
                    metrics['min'] = float(line.split(':')[1].strip().split()[0])
                elif "max" in line.lower():
                    metrics['max'] = float(line.split(':')[1].strip().split()[0])
                    
        except Exception as e:
            print(f"Lỗi khi trích xuất metric từ file {result_file}: {e}")
            
        return metrics
    
    def _plot_comparison(self, gmapping_ate, hector_ate, gmapping_rpe, hector_rpe):
        """
        Tạo biểu đồ so sánh giữa gmapping và hector
        """
        # Tạo biểu đồ cho ATE
        plt.figure(figsize=(12, 6))
        
        # ATE subplot
        plt.subplot(121)
        methods = ['GMMapping', 'Hector']
        rmse_values = [gmapping_ate['rmse'], hector_ate['rmse']]
        mean_values = [gmapping_ate['mean'], hector_ate['mean']]
        
        x = np.arange(len(methods))
        width = 0.35
        
        plt.bar(x - width/2, rmse_values, width, label='RMSE')
        plt.bar(x + width/2, mean_values, width, label='Mean')
        
        plt.xlabel('SLAM Method')
        plt.ylabel('Error (m)')
        plt.title('Absolute Trajectory Error (ATE)')
        plt.xticks(x, methods)
        plt.legend()
        
        # RPE subplot
        plt.subplot(122)
        rmse_values = [gmapping_rpe['rmse'], hector_rpe['rmse']]
        mean_values = [gmapping_rpe['mean'], hector_rpe['mean']]
        
        plt.bar(x - width/2, rmse_values, width, label='RMSE')
        plt.bar(x + width/2, mean_values, width, label='Mean')
        
        plt.xlabel('SLAM Method')
        plt.ylabel('Error (m/m)')
        plt.title('Relative Pose Error (RPE)')
        plt.xticks(x, methods)
        plt.legend()
        
        plt.tight_layout()
        
        # Lưu biểu đồ
        comparison_plot = os.path.join(self.output_dir, "slam_comparison.pdf")
        plt.savefig(comparison_plot)
        plt.savefig(os.path.join(self.output_dir, "slam_comparison.png"))
        
        print(f"Đã lưu biểu đồ so sánh vào {comparison_plot}")

def main():
    parser = argparse.ArgumentParser(description='Đánh giá độ chính xác của thuật toán SLAM trên robot thật')
    parser.add_argument('--bag', required=True,
                      help='Đường dẫn tới file rosbag')
    parser.add_argument('--method', required=True, choices=['gmapping', 'hector'],
                      help='Phương pháp SLAM cần đánh giá')
    parser.add_argument('--gt_source', default='mocap', choices=['mocap', 'vicon', 'rtabmap', 'manual', 'odom'],
                      help='Nguồn dữ liệu ground truth')
    parser.add_argument('--output_dir', default=None,
                      help='Thư mục lưu kết quả')
    
    args = parser.parse_args()
    
    # Kiểm tra file rosbag có tồn tại không
    if not os.path.isfile(args.bag):
        print(f"Lỗi: File rosbag không tồn tại: {args.bag}")
        return
    
    # Tạo đối tượng đánh giá
    evaluator = RealRobotSLAMEvaluator(args.bag, args.method, args.gt_source, args.output_dir)
    
    # Trích xuất quỹ đạo
    if evaluator.extract_trajectories():
        # Thực hiện đánh giá
        evaluator.run_evaluation()
    else:
        # Nếu không thể trích xuất quỹ đạo, thử tạo ground truth tổng hợp
        print("Thử tạo ground truth tổng hợp...")
        if evaluator.create_synthetic_ground_truth() and evaluator.extract_trajectories():
            evaluator.run_evaluation()
        else:
            print("Không thể thực hiện đánh giá do lỗi khi trích xuất quỹ đạo.")

if __name__ == "__main__":
    main()