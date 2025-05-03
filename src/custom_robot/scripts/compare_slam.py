#!/usr/bin/env python

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from evo.tools import file_interface
from evo.core import metrics, trajectory
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.tools import plot
from evo.core.trajectory import PoseTrajectory3D
import argparse
import rosbag
import tf.transformations
import yaml

def extract_trajectory_from_bag(bag_file, topic, frame_id=None, ref_frame_id=None):
    """
    Trích xuất quỹ đạo từ file rosbag
    
    Args:
        bag_file: đường dẫn tới file rosbag
        topic: topic chứa dữ liệu quỹ đạo
        frame_id: ID của frame
        ref_frame_id: ID của frame tham chiếu
        
    Returns:
        Trajectory object của evo
    """
    try:
        bag = rosbag.Bag(bag_file, 'r')
        poses = []
        timestamps = []
        
        # Kiểm tra xem topic có tồn tại không
        topics = bag.get_type_and_topic_info()[1]
        if topic not in topics:
            print(f"Topic {topic} không có trong file bag")
            print(f"Các topic có sẵn: {list(topics.keys())}")
            return None
            
        for _, msg, t in bag.read_messages(topics=topic):
            # Điều chỉnh theo loại message
            # Đây là ví dụ cho geometry_msgs/PoseStamped hoặc nav_msgs/Path
            if topic.endswith('pose'):
                # PoseStamped
                position = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                quat = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
                timestamp = msg.header.stamp.to_sec()
                poses.append(position + quat)
                timestamps.append(timestamp)
            elif topic.endswith('path') or topic.endswith('trajectory'):
                # Path message
                for pose in msg.poses:
                    position = [
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z
                    ]
                    quat = [
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z, 
                        pose.pose.orientation.w
                    ]
                    timestamp = pose.header.stamp.to_sec()
                    poses.append(position + quat)
                    timestamps.append(timestamp)
        
        # Kiểm tra xem đã thu thập được dữ liệu không
        if not poses:
            print(f"Không tìm thấy dữ liệu pose trong topic {topic}")
            return None
            
        bag.close()
        
        # Tạo đối tượng quỹ đạo
        traj = PoseTrajectory3D(
            positions_xyz=np.array([p[:3] for p in poses]),
            orientations_quat_wxyz=np.array([[p[6], p[3], p[4], p[5]] for p in poses]),  # XYZW -> WXYZ
            timestamps=np.array(timestamps)
        )
        
        # Thiết lập metadata
        traj.meta = {
            "frame_id": frame_id if frame_id else "",
            "ref_frame_id": ref_frame_id if ref_frame_id else ""
        }
        
        return traj
    except Exception as e:
        print(f"Lỗi khi trích xuất quỹ đạo từ bag: {e}")
        return None

def compute_ate(traj_ref, traj_est):
    """
    Tính toán Absolute Trajectory Error (ATE)
    
    Args:
        traj_ref: quỹ đạo tham chiếu (ground truth)
        traj_est: quỹ đạo ước tính (từ thuật toán SLAM)
        
    Returns:
        Kết quả ATE
    """
    # Đồng bộ hóa các quỹ đạo theo timestamp (chú ý: có thể mất dữ liệu)
    traj_ref, traj_est = trajectory.sync_trajectory_pairs(traj_ref, traj_est)
    
    # Tính ATE
    pose_relation = metrics.PoseRelation.translation_part
    data = (traj_ref, traj_est)
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)
    
    return ape_metric

def compute_rpe(traj_ref, traj_est, delta, delta_unit="m", all_pairs=False):
    """
    Tính toán Relative Pose Error (RPE)
    
    Args:
        traj_ref: quỹ đạo tham chiếu (ground truth)
        traj_est: quỹ đạo ước tính (từ thuật toán SLAM)
        delta: khoảng thời gian hoặc khoảng cách để tính RPE
        delta_unit: đơn vị của delta ("m" cho meters, "rad" cho radians, "f" cho frames)
        all_pairs: tính RPE cho tất cả các cặp có thể (không chỉ các cặp liên tiếp)
        
    Returns:
        Kết quả RPE
    """
    # Đồng bộ hóa các quỹ đạo theo timestamp
    traj_ref, traj_est = trajectory.sync_trajectory_pairs(traj_ref, traj_est)
    
    # Tính RPE
    pose_relation = metrics.PoseRelation.translation_part
    data = (traj_ref, traj_est)
    rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, all_pairs)
    rpe_metric.process_data(data)
    
    return rpe_metric

def plot_results(ate_results, rpe_results, slam_names, output_path=None):
    """
    Vẽ biểu đồ kết quả so sánh
    """
    # Nếu chỉ có một thuật toán SLAM, vẽ biểu đồ đơn thuần
    if len(slam_names) == 1:
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        
        # ATE Histogram
        axs[0].hist(ate_results[0].error, bins=50, alpha=0.7)
        axs[0].set_title(f'{slam_names[0]} ATE Histogram')
        axs[0].set_xlabel('Error (m)')
        axs[0].set_ylabel('Count')
        
        # RPE Histogram
        axs[1].hist(rpe_results[0].error, bins=50, alpha=0.7)
        axs[1].set_title(f'{slam_names[0]} RPE Histogram')
        axs[1].set_xlabel('Error (m)')
        axs[1].set_ylabel('Count')
        
    # Nếu có nhiều thuật toán SLAM, so sánh chúng
    else:
        fig, axs = plt.subplots(2, 2, figsize=(15, 10))
        
        # ATE Histogram
        for i, (ate, name) in enumerate(zip(ate_results, slam_names)):
            axs[0, 0].hist(ate.error, bins=50, alpha=0.5, label=name)
        axs[0, 0].set_title('ATE Histogram')
        axs[0, 0].set_xlabel('Error (m)')
        axs[0, 0].set_ylabel('Count')
        axs[0, 0].legend()
        
        # RPE Histogram
        for i, (rpe, name) in enumerate(zip(rpe_results, slam_names)):
            axs[0, 1].hist(rpe.error, bins=50, alpha=0.5, label=name)
        axs[0, 1].set_title('RPE Histogram')
        axs[0, 1].set_xlabel('Error (m)')
        axs[0, 1].set_ylabel('Count')
        axs[0, 1].legend()
        
        # Box plots
        data_ate = [ate.error for ate in ate_results]
        axs[1, 0].boxplot(data_ate)
        axs[1, 0].set_title('ATE Box Plot')
        axs[1, 0].set_xticklabels(slam_names)
        axs[1, 0].set_ylabel('Error (m)')
        
        data_rpe = [rpe.error for rpe in rpe_results]
        axs[1, 1].boxplot(data_rpe)
        axs[1, 1].set_title('RPE Box Plot')
        axs[1, 1].set_xticklabels(slam_names)
        axs[1, 1].set_ylabel('Error (m)')
    
    plt.tight_layout()
    
    # Lưu kết quả vào file nếu được chỉ định
    if output_path:
        plt.savefig(output_path)
        print(f"Biểu đồ đã được lưu vào {output_path}")
    
    # Hiển thị biểu đồ
    plt.show()

def save_results_to_file(results, slam_names, output_file):
    """
    Lưu kết quả thống kê vào file
    """
    output_data = {}
    
    for i, name in enumerate(slam_names):
        output_data[name] = {
            'ate': {
                'rmse': float(results['ate'][i].stats["rmse"]),
                'mean': float(results['ate'][i].stats["mean"]),
                'median': float(results['ate'][i].stats["median"]),
                'std': float(results['ate'][i].stats["std"]),
                'min': float(results['ate'][i].stats["min"]),
                'max': float(results['ate'][i].stats["max"])
            },
            'rpe': {
                'rmse': float(results['rpe'][i].stats["rmse"]),
                'mean': float(results['rpe'][i].stats["mean"]),
                'median': float(results['rpe'][i].stats["median"]),
                'std': float(results['rpe'][i].stats["std"]),
                'min': float(results['rpe'][i].stats["min"]),
                'max': float(results['rpe'][i].stats["max"])
            }
        }
    
    with open(output_file, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False)
    
    print(f"Kết quả đã được lưu vào {output_file}")

def print_statistics(ate_results, rpe_results, slam_names):
    """
    In kết quả thống kê ra console
    """
    for i, name in enumerate(slam_names):
        print(f"\n==== {name} ====")
        print("\nAbsolute Trajectory Error (ATE):")
        print(f"RMSE: {ate_results[i].stats['rmse']:.6f} m")
        print(f"Mean: {ate_results[i].stats['mean']:.6f} m")
        print(f"Median: {ate_results[i].stats['median']:.6f} m")
        print(f"Std: {ate_results[i].stats['std']:.6f} m")
        print(f"Min: {ate_results[i].stats['min']:.6f} m")
        print(f"Max: {ate_results[i].stats['max']:.6f} m")
        
        print("\nRelative Pose Error (RPE):")
        print(f"RMSE: {rpe_results[i].stats['rmse']:.6f} m")
        print(f"Mean: {rpe_results[i].stats['mean']:.6f} m")
        print(f"Median: {rpe_results[i].stats['median']:.6f} m")
        print(f"Std: {rpe_results[i].stats['std']:.6f} m")
        print(f"Min: {rpe_results[i].stats['min']:.6f} m")
        print(f"Max: {rpe_results[i].stats['max']:.6f} m")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="So sánh hiệu suất thuật toán SLAM dựa trên ATE và RPE")
    parser.add_argument("bag_file", help="File rosbag chứa dữ liệu quỹ đạo")
    parser.add_argument("--gt-topic", default="/ground_truth/path", help="Topic chứa ground truth")
    parser.add_argument("--gmapping-topic", default="/gmapping/trajectory", help="Topic chứa quỹ đạo từ Gmapping")
    parser.add_argument("--hector-topic", default="/hector/trajectory", help="Topic chứa quỹ đạo từ Hector")
    parser.add_argument("--slam-topic", help="Topic chứa quỹ đạo từ thuật toán SLAM (khi chỉ phân tích một thuật toán)")
    parser.add_argument("--slam-name", default="SLAM", help="Tên thuật toán SLAM (khi chỉ phân tích một thuật toán)")
    parser.add_argument("--only-one", action="store_true", help="Chỉ phân tích một thuật toán SLAM")
    parser.add_argument("--rpe-delta", type=float, default=1.0, help="Delta cho RPE (mặc định: 1.0)")
    parser.add_argument("--rpe-unit", default="m", choices=["m", "rad", "f"], help="Đơn vị delta cho RPE (mặc định: m)")
    parser.add_argument("--output", default=os.path.expanduser("~/catkin_ws/slam_results/results.yaml"), help="File lưu kết quả")
    parser.add_argument("--plot-output", help="Đường dẫn để lưu hình ảnh kết quả")
    args = parser.parse_args()
    
    # Kiểm tra file bag
    if not os.path.exists(args.bag_file):
        print(f"File bag không tồn tại: {args.bag_file}")
        sys.exit(1)
    
    # Trích xuất quỹ đạo ground truth
    print("Trích xuất quỹ đạo ground truth từ file rosbag...")
    ground_truth = extract_trajectory_from_bag(args.bag_file, args.gt_topic, frame_id="map")
    
    if not ground_truth:
        print(f"Không thể trích xuất quỹ đạo ground truth từ topic {args.gt_topic}")
        sys.exit(1)
    
    slam_trajectories = []
    slam_names = []
    
    # Nếu chỉ phân tích một thuật toán SLAM
    if args.only_one:
        if args.slam_topic:
            slam_topic = args.slam_topic
        else:
            slam_topic = args.gmapping_topic  # Mặc định là Gmapping
        
        print(f"Trích xuất quỹ đạo từ thuật toán SLAM ({args.slam_name}) từ topic {slam_topic}...")
        slam_traj = extract_trajectory_from_bag(args.bag_file, slam_topic, frame_id="map")
        
        if not slam_traj:
            print(f"Không thể trích xuất quỹ đạo từ topic {slam_topic}")
            sys.exit(1)
        
        slam_trajectories = [slam_traj]
        slam_names = [args.slam_name]
    
    # Nếu phân tích cả Gmapping và Hector SLAM
    else:
        # Trích xuất quỹ đạo Gmapping
        print("Trích xuất quỹ đạo Gmapping từ file rosbag...")
        gmapping_traj = extract_trajectory_from_bag(args.bag_file, args.gmapping_topic, frame_id="map")
        
        if not gmapping_traj:
            print(f"Không thể trích xuất quỹ đạo Gmapping từ topic {args.gmapping_topic}")
            sys.exit(1)
        
        # Trích xuất quỹ đạo Hector SLAM
        print("Trích xuất quỹ đạo Hector SLAM từ file rosbag...")
        hector_traj = extract_trajectory_from_bag(args.bag_file, args.hector_topic, frame_id="map")
        
        if not hector_traj:
            print(f"Không thể trích xuất quỹ đạo Hector từ topic {args.hector_topic}")
            sys.exit(1)
        
        slam_trajectories = [gmapping_traj, hector_traj]
        slam_names = ["Gmapping", "Hector"]
    
    # Tính ATE cho tất cả các quỹ đạo SLAM
    print("Tính toán ATE...")
    ate_results = []
    for traj in slam_trajectories:
        ate = compute_ate(ground_truth, traj)
        ate_results.append(ate)
    
    # Tính RPE cho tất cả các quỹ đạo SLAM
    print("Tính toán RPE...")
    rpe_results = []
    for traj in slam_trajectories:
        rpe = compute_rpe(ground_truth, traj, args.rpe_delta, args.rpe_unit)
        rpe_results.append(rpe)
    
    # In kết quả
    print_statistics(ate_results, rpe_results, slam_names)
    
    # Lưu kết quả vào file
    results_dir = os.path.dirname(args.output)
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    save_results_to_file(
        {'ate': ate_results, 'rpe': rpe_results}, 
        slam_names, 
        args.output
    )
    
    # Vẽ biểu đồ
    plot_results(ate_results, rpe_results, slam_names, args.plot_output)

if __name__ == "__main__":
    main()