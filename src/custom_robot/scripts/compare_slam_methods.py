#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

def load_trajectory_from_file(file_path):
    """
    Đọc dữ liệu quỹ đạo từ file TUM format
    """
    if not os.path.isfile(file_path):
        print(f"Lỗi: File không tồn tại: {file_path}")
        return None
        
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            values = line.strip().split()
            if len(values) == 8:
                data.append([float(v) for v in values])
    
    if not data:
        print(f"Không có dữ liệu trong file {file_path}")
        return None
        
    return np.array(data)

def compare_methods(gt_file, gmapping_file, hector_file, output_dir):
    """
    So sánh hai phương pháp SLAM với groundtruth
    """
    print("So sánh Gmapping và Hector SLAM...")
    
    # Tạo thư mục output nếu chưa tồn tại
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    comparison_plot = os.path.join(output_dir, f"comparison_plot_{timestamp}.pdf")
    comparison_results = os.path.join(output_dir, f"comparison_results_{timestamp}.txt")
    
    # Chạy lệnh evo_ape_traj để so sánh với ground truth
    compare_cmd = [
        "evo_ape_traj", "tum", 
        "--ref", gt_file,
        "--est", gmapping_file, hector_file,
        "--label", "Gmapping", "Hector",
        "-p", "--plot_mode", "xy",
        "--save_plot", comparison_plot,
        "--save_results", os.path.join(output_dir, f"comparison_data_{timestamp}.zip")
    ]
    
    try:
        result = subprocess.run(compare_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(result.stdout)
        
        # Lưu kết quả vào file
        with open(comparison_results, 'w') as f:
            f.write("=== SO SÁNH GMAPPING VÀ HECTOR SLAM ===\n\n")
            f.write(result.stdout)
            
            if result.stderr:
                f.write("\n=== LỖI ===\n")
                f.write(result.stderr)
        
        # Chạy lệnh evo_rpe_traj để so sánh RPE
        print("\nSo sánh Relative Pose Error (RPE)...")
        
        rpe_cmd = [
            "evo_rpe_traj", "tum", 
            "--ref", gt_file,
            "--est", gmapping_file, hector_file,
            "--label", "Gmapping", "Hector",
            "-p", "--plot_mode", "xy",
            "--save_plot", os.path.join(output_dir, f"rpe_comparison_{timestamp}.pdf"),
            "--save_results", os.path.join(output_dir, f"rpe_comparison_data_{timestamp}.zip")
        ]
        
        rpe_result = subprocess.run(rpe_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(rpe_result.stdout)
        
        # Lưu thêm kết quả RPE vào file
        with open(comparison_results, 'a') as f:
            f.write("\n\n=== SO SÁNH RELATIVE POSE ERROR (RPE) ===\n\n")
            f.write(rpe_result.stdout)
            
            if rpe_result.stderr:
                f.write("\n=== LỖI RPE ===\n")
                f.write(rpe_result.stderr)
                
        print(f"\nKết quả so sánh đã được lưu vào {comparison_results}")
        print(f"Đồ thị so sánh ATE: {comparison_plot}")
        print(f"Đồ thị so sánh RPE: {os.path.join(output_dir, f'rpe_comparison_{timestamp}.pdf')}")
        
    except Exception as e:
        print(f"Lỗi khi so sánh các phương pháp SLAM: {e}")
        return False
        
    return True

def main():
    parser = argparse.ArgumentParser(description='So sánh các phương pháp SLAM')
    parser.add_argument('--gt', required=True,
                      help='Đường dẫn tới file quỹ đạo ground truth')
    parser.add_argument('--gmapping', required=True,
                      help='Đường dẫn tới file quỹ đạo Gmapping')
    parser.add_argument('--hector', required=True,
                      help='Đường dẫn tới file quỹ đạo Hector')
    parser.add_argument('--output_dir', default=os.path.expanduser("~/catkin_ws/src/custom_robot/evaluation/results/comparison"),
                      help='Thư mục lưu kết quả')
    
    args = parser.parse_args()
    
    # Kiểm tra các file đầu vào
    for file_path, name in [(args.gt, "Ground truth"), (args.gmapping, "Gmapping"), (args.hector, "Hector")]:
        if not os.path.isfile(file_path):
            print(f"Lỗi: File {name} không tồn tại: {file_path}")
            return
    
    # So sánh các phương pháp
    compare_methods(args.gt, args.gmapping, args.hector, args.output_dir)

if __name__ == "__main__":
    main()