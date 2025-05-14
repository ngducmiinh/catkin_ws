#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import csv
import time
import signal
import statistics
import pandas as pd
import seaborn as sns

class BatchSLAMEvaluator:
    def __init__(self, iterations, slam_method, duration, output_dir=None, world_file="turtlebot3_world"):
        """
        Khởi tạo bộ đánh giá SLAM hàng loạt
        :param iterations: Số lần lặp lại kiểm tra
        :param slam_method: Phương pháp SLAM (gmapping hoặc hector)
        :param duration: Thời gian chạy mỗi lần kiểm tra (giây)
        :param output_dir: Thư mục lưu kết quả
        :param world_file: File world Gazebo sử dụng (không cần .world)
        """
        self.iterations = iterations
        self.slam_method = slam_method
        self.duration = duration
        self.world_file = world_file
        
        # Thiết lập đường dẫn output
        if output_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.expanduser(f"~/catkin_ws/src/custom_robot/evaluation/results/batch_{slam_method}_{timestamp}")
        
        self.output_dir = output_dir
        
        # Tạo thư mục output nếu chưa tồn tại
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Kết quả tổng hợp
        self.summary_file = os.path.join(self.output_dir, f"summary_{slam_method}.csv")
        
        # Kết quả chi tiết
        self.ate_results = []
        self.rpe_results = []

    def run_single_evaluation(self, iteration):
        """
        Chạy một lần mô phỏng và đánh giá
        :param iteration: Số thứ tự lần chạy
        :return: Tuple (ATE, RPE) hoặc None nếu có lỗi
        """
        print(f"\n===== BẮT ĐẦU LẦN CHẠY THỨ {iteration}/{self.iterations} =====")
        
        # Tạo thư mục cho lần chạy này
        run_dir = os.path.join(self.output_dir, f"run_{iteration}")
        if not os.path.exists(run_dir):
            os.makedirs(run_dir)
        
        # Tên cho bag file
        bag_file = os.path.join(run_dir, f"{self.slam_method}_run_{iteration}.bag")
        
        # 1. Khởi chạy Gazebo với thế giới được chỉ định
        print(f"Khởi chạy Gazebo với world: {self.world_file}...")
        gazebo_cmd = f"roslaunch turtlebot3_gazebo turtlebot3_world.launch world_name:={self.world_file}"
        gazebo_proc = subprocess.Popen(gazebo_cmd, shell=True, preexec_fn=os.setsid)
        
        # Đợi Gazebo khởi động
        time.sleep(10)  
        
        # 2. Khởi chạy SLAM
        print(f"Khởi chạy SLAM phương pháp {self.slam_method}...")
        slam_cmd = f"roslaunch custom_robot robot_slam.launch method:={self.slam_method}"
        slam_proc = subprocess.Popen(slam_cmd, shell=True, preexec_fn=os.setsid)
        
        # Đợi SLAM khởi động
        time.sleep(5)
        
        # 3. Bắt đầu ghi bag file
        print(f"Bắt đầu ghi bag file: {bag_file}")
        topics_to_record = [
            "/scan", 
            "/tf", 
            "/tf_static", 
            "/odom", 
            "/imu", 
            "/gazebo/model_states",
            f"/{self.slam_method}_slam/map",
            f"/{self.slam_method}_slam/pose"
        ]
        
        record_cmd = f"rosbag record -O {bag_file} {' '.join(topics_to_record)}"
        record_proc = subprocess.Popen(record_cmd, shell=True, preexec_fn=os.setsid)
        
        # 4. Di chuyển robot ngẫu nhiên
        print(f"Di chuyển robot ngẫu nhiên trong {self.duration} giây...")
        nav_cmd = f"roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
        nav_env = os.environ.copy()
        nav_env["TURTLEBOT3_MODEL"] = "burger"  # Đảm bảo model đúng
        
        # Sử dụng script tự động di chuyển thay vì teleop thủ công
        random_nav_cmd = f"rosrun custom_robot custom_teleop.py --random --duration {self.duration}"
        nav_proc = subprocess.Popen(random_nav_cmd, shell=True, env=nav_env, preexec_fn=os.setsid)
        
        # Đợi điều hướng hoàn thành
        print(f"Đang di chuyển robot... Vui lòng đợi {self.duration} giây")
        time.sleep(self.duration + 5)  # Thêm 5 giây để đảm bảo
        
        # 5. Dừng ghi bag file
        print("Dừng ghi bag file...")
        os.killpg(os.getpgid(record_proc.pid), signal.SIGINT)
        time.sleep(3)  # Đợi bag file đóng
        
        # 6. Dừng các quy trình khác
        print("Dừng các quy trình...")
        os.killpg(os.getpgid(nav_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(slam_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(gazebo_proc.pid), signal.SIGINT)
        
        # 7. Đợi tất cả các quy trình kết thúc
        time.sleep(5)
        
        # 8. Đánh giá SLAM
        print("Đánh giá quỹ đạo SLAM...")
        eval_cmd = f"python3 {os.path.expanduser('~/catkin_ws/src/custom_robot/scripts/evaluate_slam.py')} --bag {bag_file} --method {self.slam_method} --output_dir {run_dir}"
        eval_result = subprocess.run(eval_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if eval_result.returncode != 0:
            print(f"Lỗi khi đánh giá quỹ đạo: {eval_result.stderr}")
            return None
        
        # 9. Trích xuất kết quả đánh giá
        results_files = [f for f in os.listdir(run_dir) if f.startswith(f"results_{self.slam_method}")]
        if not results_files:
            print("Không tìm thấy file kết quả!")
            return None
        
        results_file = os.path.join(run_dir, results_files[0])
        
        # Đọc kết quả từ file và trích xuất chỉ số ATE và RPE
        with open(results_file, 'r') as f:
            content = f.read()
            
        # Trích xuất ATE RMSE
        ate_section = content.split("==== ABSOLUTE TRAJECTORY ERROR")[1].split("====")[0] if "ABSOLUTE TRAJECTORY ERROR" in content else ""
        ate_rmse = None
        for line in ate_section.split('\n'):
            if "rmse" in line.lower():
                ate_rmse = float(line.split(':')[1].strip().split()[0])
                break
        
        # Trích xuất RPE RMSE
        rpe_section = content.split("==== RELATIVE POSE ERROR")[1] if "RELATIVE POSE ERROR" in content else ""
        rpe_rmse = None
        for line in rpe_section.split('\n'):
            if "rmse" in line.lower():
                rpe_rmse = float(line.split(':')[1].strip().split()[0])
                break
                
        print(f"Kết quả lần chạy thứ {iteration}: ATE RMSE = {ate_rmse}, RPE RMSE = {rpe_rmse}")
        return (ate_rmse, rpe_rmse)

    def run_batch_evaluation(self):
        """
        Chạy đánh giá hàng loạt với số lần lặp đã chỉ định
        """
        print(f"===== BẮT ĐẦU ĐÁNH GIÁ HÀNG LOẠT SLAM {self.slam_method.upper()} =====")
        print(f"Số lần lặp: {self.iterations}, Thời gian mỗi lần: {self.duration} giây")
        
        for i in range(1, self.iterations + 1):
            # Chạy một lần đánh giá
            result = self.run_single_evaluation(i)
            
            if result:
                ate, rpe = result
                self.ate_results.append(ate)
                self.rpe_results.append(rpe)
        
        # Tính toán thống kê sau khi chạy xong tất cả các lần
        if self.ate_results and self.rpe_results:
            self.generate_statistics()
        else:
            print("Không có đủ dữ liệu để tạo thống kê!")
            
        print(f"\n===== KẾT THÚC ĐÁNH GIÁ HÀNG LOẠT SLAM {self.slam_method.upper()} =====")

    def generate_statistics(self):
        """
        Tạo thống kê và trực quan hóa từ kết quả các lần chạy
        """
        # Tính các chỉ số thống kê
        ate_mean = statistics.mean(self.ate_results)
        ate_median = statistics.median(self.ate_results)
        ate_stddev = statistics.stdev(self.ate_results) if len(self.ate_results) > 1 else 0
        ate_min = min(self.ate_results)
        ate_max = max(self.ate_results)
        
        rpe_mean = statistics.mean(self.rpe_results)
        rpe_median = statistics.median(self.rpe_results)
        rpe_stddev = statistics.stdev(self.rpe_results) if len(self.rpe_results) > 1 else 0
        rpe_min = min(self.rpe_results)
        rpe_max = max(self.rpe_results)
        
        # Lưu kết quả thống kê vào CSV
        with open(self.summary_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Metric', 'Mean', 'Median', 'StdDev', 'Min', 'Max'])
            writer.writerow(['ATE_RMSE', ate_mean, ate_median, ate_stddev, ate_min, ate_max])
            writer.writerow(['RPE_RMSE', rpe_mean, rpe_median, rpe_stddev, rpe_min, rpe_max])
            
            # Lưu chi tiết từng lần chạy
            writer.writerow([])
            writer.writerow(['Run', 'ATE_RMSE', 'RPE_RMSE'])
            for i, (ate, rpe) in enumerate(zip(self.ate_results, self.rpe_results), 1):
                writer.writerow([i, ate, rpe])
        
        print(f"\n===== THỐNG KÊ CHO {self.slam_method.upper()} =====")
        print(f"ATE RMSE: mean={ate_mean:.6f}, median={ate_median:.6f}, stddev={ate_stddev:.6f}, min={ate_min:.6f}, max={ate_max:.6f}")
        print(f"RPE RMSE: mean={rpe_mean:.6f}, median={rpe_median:.6f}, stddev={rpe_stddev:.6f}, min={rpe_min:.6f}, max={rpe_max:.6f}")
        print(f"Kết quả chi tiết đã được lưu vào: {self.summary_file}")
        
        # Tạo biểu đồ
        self.create_plots()

    def create_plots(self):
        """
        Tạo các biểu đồ trực quan hóa kết quả
        """
        # Tạo DataFrame
        runs = list(range(1, len(self.ate_results) + 1))
        df = pd.DataFrame({
            'Run': runs,
            'ATE RMSE': self.ate_results,
            'RPE RMSE': self.rpe_results
        })
        
        # 1. Biểu đồ đường cho ATE và RPE qua các lần chạy
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        sns.lineplot(data=df, x='Run', y='ATE RMSE', marker='o')
        plt.title(f'{self.slam_method.upper()} - ATE RMSE qua các lần chạy')
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        sns.lineplot(data=df, x='Run', y='RPE RMSE', marker='o', color='orange')
        plt.title(f'{self.slam_method.upper()} - RPE RMSE qua các lần chạy')
        plt.grid(True)
        
        plt.tight_layout()
        line_plot_file = os.path.join(self.output_dir, f"{self.slam_method}_metrics_by_run.png")
        plt.savefig(line_plot_file)
        
        # 2. Biểu đồ boxplot để thấy phân phối
        plt.figure(figsize=(10, 6))
        
        # Convert to long format for seaborn
        df_long = pd.melt(df, id_vars=['Run'], value_vars=['ATE RMSE', 'RPE RMSE'], 
                          var_name='Metric', value_name='RMSE')
        
        sns.boxplot(x='Metric', y='RMSE', data=df_long)
        plt.title(f'{self.slam_method.upper()} - Phân phối lỗi qua {len(runs)} lần chạy')
        plt.grid(True, axis='y')
        
        boxplot_file = os.path.join(self.output_dir, f"{self.slam_method}_error_distribution.png")
        plt.savefig(boxplot_file)
        
        # 3. Biểu đồ histogram
        plt.figure(figsize=(12, 5))
        
        plt.subplot(1, 2, 1)
        sns.histplot(self.ate_results, kde=True)
        plt.title(f'{self.slam_method.upper()} - Histogram ATE RMSE')
        plt.xlabel('ATE RMSE')
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        sns.histplot(self.rpe_results, kde=True, color='orange')
        plt.title(f'{self.slam_method.upper()} - Histogram RPE RMSE')
        plt.xlabel('RPE RMSE')
        plt.grid(True)
        
        plt.tight_layout()
        hist_file = os.path.join(self.output_dir, f"{self.slam_method}_error_histogram.png")
        plt.savefig(hist_file)
        
        print(f"Đã lưu biểu đồ phân tích tại: {self.output_dir}")

def compare_methods(gmapping_summary, hector_summary, output_dir):
    """
    So sánh kết quả giữa gmapping và hector
    """
    # Đọc dữ liệu từ các file CSV
    gmapping_data = pd.read_csv(gmapping_summary, skiprows=2, nrows=1, header=None)
    hector_data = pd.read_csv(hector_summary, skiprows=2, nrows=1, header=None)
    
    gmapping_runs = pd.read_csv(gmapping_summary, skiprows=5, header=None)
    hector_runs = pd.read_csv(hector_summary, skiprows=5, header=None)
    
    # Tạo DataFrame cho biểu đồ so sánh
    comparison_data = pd.DataFrame({
        'Method': ['GMMapping', 'Hector'],
        'ATE RMSE Mean': [gmapping_data.iloc[0, 1], hector_data.iloc[0, 1]],
        'ATE RMSE StdDev': [gmapping_data.iloc[0, 3], hector_data.iloc[0, 3]],
        'RPE RMSE Mean': [gmapping_data.iloc[1, 1], hector_data.iloc[1, 1]],
        'RPE RMSE StdDev': [gmapping_data.iloc[1, 3], hector_data.iloc[1, 3]]
    })
    
    # Tạo DataFrame cho kết quả chi tiết
    gmapping_detail = gmapping_runs.copy()
    gmapping_detail.columns = ['Run', 'GMMapping ATE', 'GMMapping RPE']
    
    hector_detail = hector_runs.copy() 
    hector_detail.columns = ['Run', 'Hector ATE', 'Hector RPE']
    
    detail_data = pd.merge(gmapping_detail, hector_detail, on='Run')
    
    # 1. Biểu đồ so sánh giữa các phương pháp
    plt.figure(figsize=(14, 7))
    
    # ATE subplot
    plt.subplot(1, 2, 1)
    bars = plt.bar(['GMMapping', 'Hector'], 
            [comparison_data['ATE RMSE Mean'][0], comparison_data['ATE RMSE Mean'][1]],
            yerr=[comparison_data['ATE RMSE StdDev'][0], comparison_data['ATE RMSE StdDev'][1]],
            capsize=10)
    plt.ylabel('RMSE')
    plt.title('So sánh Absolute Trajectory Error (ATE)')
    plt.grid(axis='y')
    
    # Add values on top of bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.0005,
                f'{height:.4f}', ha='center', va='bottom', rotation=0)
    
    # RPE subplot
    plt.subplot(1, 2, 2)
    bars = plt.bar(['GMMapping', 'Hector'], 
            [comparison_data['RPE RMSE Mean'][0], comparison_data['RPE RMSE Mean'][1]],
            yerr=[comparison_data['RPE RMSE StdDev'][0], comparison_data['RPE RMSE StdDev'][1]],
            capsize=10, color='orange')
    plt.ylabel('RMSE')
    plt.title('So sánh Relative Pose Error (RPE)')
    plt.grid(axis='y')
    
    # Add values on top of bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.0005,
                f'{height:.4f}', ha='center', va='bottom', rotation=0)
    
    plt.tight_layout()
    comparison_file = os.path.join(output_dir, "gmapping_vs_hector_comparison.png")
    plt.savefig(comparison_file)
    
    # 2. Biểu đồ đường so sánh qua các lần chạy
    plt.figure(figsize=(14, 10))
    
    # ATE subplot
    plt.subplot(2, 1, 1)
    plt.plot(detail_data['Run'], detail_data['GMMapping ATE'], 'b-o', label='GMMapping')
    plt.plot(detail_data['Run'], detail_data['Hector ATE'], 'r-s', label='Hector')
    plt.title('Absolute Trajectory Error (ATE) qua các lần chạy')
    plt.xlabel('Lần chạy')
    plt.ylabel('ATE RMSE')
    plt.grid(True)
    plt.legend()
    
    # RPE subplot
    plt.subplot(2, 1, 2)
    plt.plot(detail_data['Run'], detail_data['GMMapping RPE'], 'b-o', label='GMMapping')
    plt.plot(detail_data['Run'], detail_data['Hector RPE'], 'r-s', label='Hector')
    plt.title('Relative Pose Error (RPE) qua các lần chạy')
    plt.xlabel('Lần chạy')
    plt.ylabel('RPE RMSE')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    runs_file = os.path.join(output_dir, "gmapping_vs_hector_by_run.png")
    plt.savefig(runs_file)
    
    # 3. Lưu kết quả so sánh vào file
    with open(os.path.join(output_dir, "comparison_results.txt"), 'w') as f:
        f.write("===== SO SÁNH GMAPPING VÀ HECTOR SLAM =====\n\n")
        
        f.write("== ABSOLUTE TRAJECTORY ERROR (ATE) ==\n")
        f.write(f"GMMapping mean: {comparison_data['ATE RMSE Mean'][0]:.6f} ± {comparison_data['ATE RMSE StdDev'][0]:.6f}\n")
        f.write(f"Hector mean: {comparison_data['ATE RMSE Mean'][1]:.6f} ± {comparison_data['ATE RMSE StdDev'][1]:.6f}\n")
        if comparison_data['ATE RMSE Mean'][0] < comparison_data['ATE RMSE Mean'][1]:
            f.write("GMMapping có ATE thấp hơn (tốt hơn).\n\n")
        else:
            f.write("Hector có ATE thấp hơn (tốt hơn).\n\n")
        
        f.write("== RELATIVE POSE ERROR (RPE) ==\n")
        f.write(f"GMMapping mean: {comparison_data['RPE RMSE Mean'][0]:.6f} ± {comparison_data['RPE RMSE StdDev'][0]:.6f}\n")
        f.write(f"Hector mean: {comparison_data['RPE RMSE Mean'][1]:.6f} ± {comparison_data['RPE RMSE StdDev'][1]:.6f}\n")
        if comparison_data['RPE RMSE Mean'][0] < comparison_data['RPE RMSE Mean'][1]:
            f.write("GMMapping có RPE thấp hơn (tốt hơn).\n\n")
        else:
            f.write("Hector có RPE thấp hơn (tốt hơn).\n\n")
        
        f.write("== KẾT LUẬN TỔNG THỂ ==\n")
        gmapping_total = comparison_data['ATE RMSE Mean'][0] + comparison_data['RPE RMSE Mean'][0]
        hector_total = comparison_data['ATE RMSE Mean'][1] + comparison_data['RPE RMSE Mean'][1]
        
        if gmapping_total < hector_total:
            f.write("GMMapping có độ chính xác tổng thể cao hơn trong điều kiện kiểm tra này.")
        else:
            f.write("Hector có độ chính xác tổng thể cao hơn trong điều kiện kiểm tra này.")
    
    print(f"Đã lưu kết quả so sánh vào: {output_dir}")
    return os.path.join(output_dir, "comparison_results.txt")

def main():
    parser = argparse.ArgumentParser(description='Đánh giá SLAM hàng loạt (batch) trong môi trường mô phỏng')
    parser.add_argument('--iterations', type=int, default=5,
                      help='Số lần lặp lại kiểm tra (mặc định: 5)')
    parser.add_argument('--method', choices=['gmapping', 'hector', 'both'],
                      default='both', help='Phương pháp SLAM cần đánh giá (mặc định: both)')
    parser.add_argument('--duration', type=int, default=60,
                      help='Thời gian chạy mỗi lần kiểm tra, tính bằng giây (mặc định: 60)')
    parser.add_argument('--output_dir', default=None,
                      help='Thư mục lưu kết quả')
    parser.add_argument('--world', default="turtlebot3_world",
                      help='File world Gazebo (mặc định: turtlebot3_world)')
    
    args = parser.parse_args()
    
    # Tạo thư mục output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output_dir is None:
        args.output_dir = os.path.expanduser(f"~/catkin_ws/src/custom_robot/evaluation/results/batch_{timestamp}")
    
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    # Đánh giá theo phương pháp được chọn
    if args.method == 'both' or args.method == 'gmapping':
        gmapping_dir = os.path.join(args.output_dir, "gmapping")
        gmapping_evaluator = BatchSLAMEvaluator(args.iterations, 'gmapping', args.duration, gmapping_dir, args.world)
        gmapping_evaluator.run_batch_evaluation()
    
    if args.method == 'both' or args.method == 'hector':
        hector_dir = os.path.join(args.output_dir, "hector")
        hector_evaluator = BatchSLAMEvaluator(args.iterations, 'hector', args.duration, hector_dir, args.world)
        hector_evaluator.run_batch_evaluation()
    
    # So sánh kết quả nếu đánh giá cả hai phương pháp
    if args.method == 'both':
        gmapping_summary = os.path.join(gmapping_dir, "summary_gmapping.csv")
        hector_summary = os.path.join(hector_dir, "summary_hector.csv")
        
        if os.path.exists(gmapping_summary) and os.path.exists(hector_summary):
            comparison_file = compare_methods(gmapping_summary, hector_summary, args.output_dir)
            print(f"\n===== SO SÁNH GMAPPING VÀ HECTOR SLAM =====")
            print(f"Kết quả so sánh đã được lưu vào: {comparison_file}")
    
if __name__ == "__main__":
    main()