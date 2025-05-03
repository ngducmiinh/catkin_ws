#!/usr/bin/env python3

import argparse
import os
import subprocess
import matplotlib.pyplot as plt
from datetime import datetime
import shutil

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
    
    # So sánh cả hai phương pháp cùng lúc
    comparison_cmd = [
        "evo_ape", "tum", gt_file, gmapping_file, hector_file,
        "-p", "--plot_mode", "xy",
        "--save_plot", comparison_plot,
        "--save_results", os.path.join(output_dir, f"comparison_data_{timestamp}.zip"),
        "--align", "--correct_scale"
    ]
    
    # Tạo file để lưu kết quả
    results_file = open(comparison_results, "w")
    
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
    except Exception as e:
        print(f"Lỗi khi đánh giá Hector: {e}")
    
    # So sánh cả hai phương pháp
    print("\n[3/3] So sánh Gmapping và Hector SLAM...")
    try:
        comparison_result = subprocess.run(
            comparison_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        print(comparison_result.stdout)
        if comparison_result.stderr and "error" in comparison_result.stderr.lower():
            print(f"Lỗi khi so sánh: {comparison_result.stderr}")
            
        results_file.write("==== SO SÁNH CẢ HAI PHƯƠNG PHÁP ====\n")
        results_file.write(comparison_result.stdout)
    except Exception as e:
        print(f"Lỗi khi so sánh các phương pháp SLAM: {e}")
    
    results_file.close()
    
    print(f"\nKết quả so sánh đã được lưu vào {comparison_results}")
    print(f"Đồ thị so sánh: {comparison_plot}")
    
    return True

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