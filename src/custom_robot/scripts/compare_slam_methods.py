#!/usr/bin/env python3

import argparse
import os
import subprocess
import matplotlib.pyplot as plt
from datetime import datetime
import shutil
import numpy as np
import pandas as pd

def check_dependencies():
    """
    Kiểm tra các công cụ evo được cài đặt
    """
    tools = ["evo_ape", "evo_rpe"]
    missing_tools = []
    
    for tool in tools:
        if shutil.which(tool) is None:
            missing_tools.append(tool)
            
    if missing_tools:
        print("Lỗi: Các công cụ sau chưa được cài đặt:")
        for tool in missing_tools:
            print(f"  - {tool}")
        print("\nVui lòng cài đặt gói evo bằng lệnh:")
        print("  pip install evo")
        return False
    
    return True

def compute_statistics(results_file, gmapping_results, hector_results):
    """
    Tạo bảng so sánh thống kê giữa hai phương pháp
    """
    results_file.write("==== SO SÁNH THỐNG KÊ GIỮA HAI PHƯƠNG PHÁP ====\n\n")
    results_file.write(f"{'Metric':<10} | {'Gmapping':<15} | {'Hector SLAM':<15} | {'Tốt hơn':<10}\n")
    results_file.write("-" * 56 + "\n")
    
    # Các metric để so sánh
    metrics = ['max', 'mean', 'median', 'min', 'rmse', 'sse', 'std']
    
    for metric in metrics:
        if metric in gmapping_results and metric in hector_results:
            g_value = gmapping_results[metric]
            h_value = hector_results[metric]
            
            # Xác định phương pháp nào tốt hơn
            if metric in ['max', 'mean', 'median', 'rmse', 'sse', 'std']:
                better = 'Gmapping' if g_value < h_value else 'Hector'
                if abs(g_value - h_value) < 0.000001:  # Nếu gần như bằng nhau
                    better = 'Tương đương'
            else:  # 'min'
                better = 'Gmapping' if g_value > h_value else 'Hector'
                if abs(g_value - h_value) < 0.000001:
                    better = 'Tương đương'
                    
            results_file.write(f"{metric:<10} | {g_value:<15.6f} | {h_value:<15.6f} | {better:<10}\n")
    
    results_file.write("\n")

def compare_methods(gt_file, gmapping_file, hector_file, output_dir=None):
    """
    So sánh hai phương pháp SLAM với groundtruth
    """
    print("So sánh Gmapping và Hector SLAM...")
    
    # Kiểm tra xem các file có tồn tại không
    files_to_check = [gt_file, gmapping_file, hector_file]
    for f in files_to_check:
        if not os.path.isfile(f):
            print(f"Lỗi: File không tồn tại: {f}")
            return False
    
    # Kiểm tra các công cụ cần thiết đã được cài đặt
    if not check_dependencies():
        return False
    
    # Thiết lập thư mục output
    if output_dir is None:
        # Sử dụng thư mục chứa file gmapping làm output
        output_dir = os.path.dirname(gmapping_file)
    
    # Đảm bảo thư mục output tồn tại
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    comparison_plot = os.path.join(output_dir, f"comparison_plot_{timestamp}.pdf")
    comparison_results = os.path.join(output_dir, f"comparison_results_{timestamp}.txt")
    
    # So sánh cả hai phương pháp với ground truth (ATE)
    gmapping_ate_cmd = [
        "evo_ape", "tum", gt_file, gmapping_file,
        "-p", "--plot_mode", "xy",
        "--save_plot", os.path.join(output_dir, f"gmapping_ate_{timestamp}.pdf"),
        "--save_results", os.path.join(output_dir, f"gmapping_ate_{timestamp}.zip"),
        "--align", "--correct_scale"
    ]
    
    hector_ate_cmd = [
        "evo_ape", "tum", gt_file, hector_file,
        "-p", "--plot_mode", "xy",
        "--save_plot", os.path.join(output_dir, f"hector_ate_{timestamp}.pdf"),
        "--save_results", os.path.join(output_dir, f"hector_ate_{timestamp}.zip"),
        "--align", "--correct_scale"
    ]
    
    # Tạo file để lưu kết quả
    results_file = open(comparison_results, "w")
    
    # Lưu trữ kết quả để so sánh
    gmapping_stats = {}
    hector_stats = {}
    
    # Chạy đánh giá cho Gmapping
    print("\n[1/3] Đánh giá Gmapping...")
    try:
        gmapping_result = subprocess.run(
            gmapping_ate_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        print(gmapping_result.stdout)
        if gmapping_result.stderr and "error" in gmapping_result.stderr.lower():
            print(f"Lỗi Gmapping: {gmapping_result.stderr}")
            
        results_file.write("==== GMAPPING ATE ====\n")
        results_file.write(gmapping_result.stdout)
        results_file.write("\n\n")
        
        # Trích xuất các thông số thống kê
        for line in gmapping_result.stdout.split('\n'):
            if ':' not in line and len(line.split()) == 2:
                parts = line.split()
                if len(parts) == 2:
                    try:
                        gmapping_stats[parts[0].strip()] = float(parts[1])
                    except:
                        pass
    except Exception as e:
        print(f"Lỗi khi đánh giá Gmapping: {e}")
    
    # Chạy đánh giá cho Hector
    print("\n[2/3] Đánh giá Hector SLAM...")
    try:
        hector_result = subprocess.run(
            hector_ate_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        print(hector_result.stdout)
        if hector_result.stderr and "error" in hector_result.stderr.lower():
            print(f"Lỗi Hector: {hector_result.stderr}")
            
        results_file.write("==== HECTOR SLAM ATE ====\n")
        results_file.write(hector_result.stdout)
        results_file.write("\n\n")
        
        # Trích xuất các thông số thống kê
        for line in hector_result.stdout.split('\n'):
            if ':' not in line and len(line.split()) == 2:
                parts = line.split()
                if len(parts) == 2:
                    try:
                        hector_stats[parts[0].strip()] = float(parts[1])
                    except:
                        pass
    except Exception as e:
        print(f"Lỗi khi đánh giá Hector: {e}")
    
    # So sánh và tạo bảng thống kê
    print("\n[3/3] So sánh kết quả của Gmapping và Hector SLAM...")
    try:
        # Tạo bảng so sánh thống kê
        compute_statistics(results_file, gmapping_stats, hector_stats)
        
        # Tạo đồ thị so sánh thủ công
        create_comparison_plot(
            output_dir, 
            comparison_plot, 
            gmapping_file, 
            hector_file, 
            gt_file,
            timestamp
        )
        
    except Exception as e:
        print(f"Lỗi khi so sánh các phương pháp SLAM: {e}")
    
    results_file.close()
    
    print(f"\nKết quả so sánh đã được lưu vào {comparison_results}")
    print(f"Đồ thị so sánh: {comparison_plot}")
    
    return True

def create_comparison_plot(output_dir, plot_file, gmapping_file, hector_file, gt_file, timestamp):
    """
    Tạo đồ thị so sánh hai phương pháp SLAM
    """
    # Tạo một đồ thị so sánh
    trajectories_plot_cmd = [
        "evo_traj", "tum", "--plot_mode", "xy",
        "--ref", gt_file,
        gmapping_file, hector_file,
        "--save_plot", os.path.join(output_dir, f"trajectories_{timestamp}.pdf"),
        "--save_results", os.path.join(output_dir, f"trajectories_data_{timestamp}.zip"),
        "--align", "--correct_scale",
        "--labels", "Ground Truth", "Gmapping", "Hector"
    ]
    
    try:
        subprocess.run(
            trajectories_plot_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        print(f"Đồ thị quỹ đạo đã được tạo: {os.path.join(output_dir, f'trajectories_{timestamp}.pdf')}")
        
        return True
    except Exception as e:
        print(f"Lỗi khi tạo đồ thị so sánh: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='So sánh phương pháp SLAM Gmapping và Hector')
    parser.add_argument('--gt', required=True,
                        help='Đường dẫn tới file quỹ đạo ground truth (định dạng TUM)')
    parser.add_argument('--gmapping', required=True,
                        help='Đường dẫn tới file quỹ đạo Gmapping (định dạng TUM)')
    parser.add_argument('--hector', required=True,
                        help='Đường dẫn tới file quỹ đạo Hector SLAM (định dạng TUM)')
    parser.add_argument('--output_dir', default=None,
                        help='Thư mục lưu kết quả')
    
    args = parser.parse_args()
    
    compare_methods(args.gt, args.gmapping, args.hector, args.output_dir)

if __name__ == "__main__":
    main()