#!/usr/bin/env python3

import os
import sys
import time
import psutil
import argparse
import subprocess
import signal
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import csv
import re

class SLAMResourceMonitor:
    def __init__(self, slam_method, duration=60, interval=0.5, output_dir=None):
        """
        Khởi tạo monitor tài nguyên cho SLAM
        :param slam_method: Phương pháp SLAM (gmapping hoặc hector)
        :param duration: Thời gian giám sát (giây)
        :param interval: Khoảng thời gian lấy mẫu (giây)
        :param output_dir: Thư mục lưu kết quả
        """
        self.slam_method = slam_method
        self.duration = duration
        self.interval = interval
        
        # Thiết lập đường dẫn output
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if output_dir is None:
            output_dir = os.path.expanduser(f"~/catkin_ws/src/custom_robot/evaluation/results/resources")
        
        self.output_dir = output_dir
        
        # Tạo thư mục nếu chưa tồn tại
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # File paths cho output
        self.data_file = os.path.join(self.output_dir, f"{self.slam_method}_resources_{timestamp}.csv")
        self.plot_file = os.path.join(self.output_dir, f"{self.slam_method}_resources_{timestamp}.pdf")
        self.summary_file = os.path.join(self.output_dir, f"{self.slam_method}_summary_{timestamp}.txt")
        
        # Tên node dựa trên phương pháp SLAM
        if slam_method == 'gmapping':
            self.node_pattern = r"slam_gmapping"
        elif slam_method == 'hector':
            self.node_pattern = r"hector_slam"
        elif slam_method == 'cartographer':
            self.node_pattern = r"cartographer_node"
        elif slam_method == 'rtabmap':
            self.node_pattern = r"rtabmap"
        else:
            self.node_pattern = slam_method
        
        # Lưu trữ dữ liệu
        self.timestamps = []
        self.cpu_data = []
        self.mem_data = []
        self.mem_mb_data = []
        self.monitored_pids = set()
    
    def find_slam_processes(self):
        """
        Tìm tất cả các quy trình liên quan đến SLAM
        """
        slam_pids = set()
        
        # Lấy danh sách tất cả các quy trình
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # Kiểm tra nếu tên quy trình khớp với mẫu của SLAM
                cmd_str = " ".join(proc.info['cmdline']) if proc.info['cmdline'] else ""
                if re.search(self.node_pattern, cmd_str, re.IGNORECASE) or \
                   re.search(self.slam_method, proc.info['name'] or "", re.IGNORECASE) or \
                   re.search(self.node_pattern, proc.info['name'] or "", re.IGNORECASE):
                    slam_pids.add(proc.info['pid'])
                    print(f"Phát hiện quy trình SLAM: {proc.info['pid']} - {proc.info['name']}")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
                
        # Nếu không tìm thấy quy trình, thử tìm qua rosnode
        if not slam_pids:
            try:
                # Liệt kê tất cả các node ROS đang chạy
                rosnode_list = subprocess.check_output(["rosnode", "list"]).decode().strip().split("\n")
                
                # Tìm node liên quan đến SLAM
                for node in rosnode_list:
                    if re.search(self.node_pattern, node, re.IGNORECASE) or \
                       re.search(self.slam_method, node, re.IGNORECASE):
                        # Lấy thông tin chi tiết về node
                        node_info = subprocess.check_output(["rosnode", "info", node]).decode()
                        pid_match = re.search(r"Pid: (\d+)", node_info)
                        if pid_match:
                            slam_pids.add(int(pid_match.group(1)))
                            print(f"Phát hiện node ROS SLAM: {node} (PID: {pid_match.group(1)})")
            except (subprocess.SubprocessError, ValueError, IndexError) as e:
                print(f"Lỗi khi tìm node ROS: {e}")
        
        return slam_pids
    
    def monitor_resources(self):
        """
        Theo dõi tài nguyên CPU và RAM của các quy trình SLAM
        """
        print(f"Bắt đầu giám sát tài nguyên cho {self.slam_method}...")
        
        # Chuẩn bị file CSV để ghi dữ liệu
        with open(self.data_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'CPU (%)', 'Memory (%)', 'Memory (MB)', 'PIDs'])
        
        start_time = time.time()
        end_time = start_time + self.duration
        
        # Lặp đến khi hết thời gian
        while time.time() < end_time:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Tìm các quy trình SLAM
            slam_pids = self.find_slam_processes()
            if not slam_pids:
                print(f"Không tìm thấy quy trình SLAM ({self.slam_method}), đang thử lại...")
                time.sleep(self.interval)
                continue
            
            self.monitored_pids.update(slam_pids)
            
            # Tính tổng CPU và RAM
            total_cpu_percent = 0
            total_memory_percent = 0
            total_memory_mb = 0
            
            for pid in slam_pids:
                try:
                    process = psutil.Process(pid)
                    process_cpu = process.cpu_percent(interval=0.1) / psutil.cpu_count()  # Chia cho số lõi để có % thực tế
                    process_memory = process.memory_percent()
                    process_memory_mb = process.memory_info().rss / 1024 / 1024  # Chuyển đổi bytes thành MB
                    
                    total_cpu_percent += process_cpu
                    total_memory_percent += process_memory
                    total_memory_mb += process_memory_mb
                    
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    # Quy trình có thể đã kết thúc hoặc không có quyền truy cập
                    continue
            
            # Lưu dữ liệu
            self.timestamps.append(elapsed_time)
            self.cpu_data.append(total_cpu_percent)
            self.mem_data.append(total_memory_percent)
            self.mem_mb_data.append(total_memory_mb)
            
            # Ghi vào file CSV
            with open(self.data_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    elapsed_time, 
                    total_cpu_percent,
                    total_memory_percent,
                    total_memory_mb,
                    ",".join(map(str, slam_pids))
                ])
            
            # In ra thông tin hiện tại
            print(f"[{elapsed_time:.1f}s] CPU: {total_cpu_percent:.1f}%, RAM: {total_memory_percent:.2f}% ({total_memory_mb:.1f} MB)")
            
            # Ngủ đến lần lấy mẫu tiếp theo
            time.sleep(self.interval)
        
        print(f"\nĐã hoàn thành giám sát tài nguyên cho {self.slam_method}.")
        print(f"Các quy trình đã được giám sát: {self.monitored_pids}")
        
        # Tạo biểu đồ và tổng kết
        self.generate_plot()
        self.write_summary()
    
    def generate_plot(self):
        """
        Tạo biểu đồ từ dữ liệu đã thu thập
        """
        if not self.timestamps:
            print("Không có dữ liệu để tạo biểu đồ!")
            return
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Biểu đồ CPU
        ax1.plot(self.timestamps, self.cpu_data, 'b-', label='CPU (%)')
        ax1.set_ylabel('CPU Usage (%)')
        ax1.set_title(f'{self.slam_method} CPU Usage')
        ax1.grid(True)
        ax1.legend()
        
        # Biểu đồ RAM
        ax2.plot(self.timestamps, self.mem_mb_data, 'r-', label='RAM (MB)')
        ax2_twin = ax2.twinx()
        ax2_twin.plot(self.timestamps, self.mem_data, 'g--', label='RAM (%)')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('RAM Usage (MB)')
        ax2_twin.set_ylabel('RAM Usage (%)')
        ax2.set_title(f'{self.slam_method} Memory Usage')
        ax2.grid(True)
        
        # Kết hợp các legend từ hai trục y
        lines1, labels1 = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
        
        fig.tight_layout()
        
        # Lưu biểu đồ
        plt.savefig(self.plot_file)
        print(f"Đã lưu biểu đồ tài nguyên vào: {self.plot_file}")
        
        # Lưu biểu đồ dạng PNG để dễ xem
        plt.savefig(self.plot_file.replace('.pdf', '.png'))
    
    def write_summary(self):
        """
        Viết báo cáo tổng kết về sử dụng tài nguyên
        """
        if not self.cpu_data or not self.mem_data:
            print("Không có dữ liệu để tạo báo cáo!")
            return
        
        with open(self.summary_file, 'w') as f:
            f.write(f"=== BÁO CÁO TÀI NGUYÊN CHO {self.slam_method.upper()} ===\n\n")
            f.write(f"Thời gian giám sát: {self.duration} giây\n")
            f.write(f"Khoảng thời gian lấy mẫu: {self.interval} giây\n")
            f.write(f"Số lượng mẫu: {len(self.timestamps)}\n")
            f.write(f"Các PID đã giám sát: {self.monitored_pids}\n\n")
            
            f.write("--- CPU Usage (%) ---\n")
            f.write(f"Trung bình: {np.mean(self.cpu_data):.2f}%\n")
            f.write(f"Cao nhất: {np.max(self.cpu_data):.2f}%\n")
            f.write(f"Thấp nhất: {np.min(self.cpu_data):.2f}%\n")
            f.write(f"Độ lệch chuẩn: {np.std(self.cpu_data):.2f}%\n\n")
            
            f.write("--- RAM Usage ---\n")
            f.write(f"Trung bình: {np.mean(self.mem_data):.3f}% ({np.mean(self.mem_mb_data):.2f} MB)\n")
            f.write(f"Cao nhất: {np.max(self.mem_data):.3f}% ({np.max(self.mem_mb_data):.2f} MB)\n")
            f.write(f"Thấp nhất: {np.min(self.mem_data):.3f}% ({np.min(self.mem_mb_data):.2f} MB)\n")
            f.write(f"Độ lệch chuẩn: {np.std(self.mem_data):.3f}% ({np.std(self.mem_mb_data):.2f} MB)\n\n")
            
            # Tính tốc độ tăng sử dụng bộ nhớ
            if len(self.mem_mb_data) > 10:
                # Sử dụng polyfit để tìm đường thẳng phù hợp với dữ liệu
                slope, _ = np.polyfit(self.timestamps, self.mem_mb_data, 1)
                f.write(f"Tốc độ tăng RAM: {slope:.3f} MB/giây\n\n")
            
            # Tính thời điểm sử dụng tài nguyên cao nhất
            max_cpu_idx = np.argmax(self.cpu_data)
            max_mem_idx = np.argmax(self.mem_mb_data)
            
            f.write(f"Thời điểm sử dụng CPU cao nhất: {self.timestamps[max_cpu_idx]:.2f} giây\n")
            f.write(f"Thời điểm sử dụng RAM cao nhất: {self.timestamps[max_mem_idx]:.2f} giây\n")
        
        print(f"Đã lưu báo cáo tổng kết vào: {self.summary_file}")

def main():
    parser = argparse.ArgumentParser(description='Giám sát tài nguyên cho thuật toán SLAM')
    parser.add_argument('--method', required=True,
                      help='Phương pháp SLAM (gmapping, hector, cartographer, rtabmap, ...)')
    parser.add_argument('--duration', type=int, default=60,
                      help='Thời gian giám sát (giây)')
    parser.add_argument('--interval', type=float, default=0.5,
                      help='Khoảng thời gian lấy mẫu (giây)')
    parser.add_argument('--output_dir', default=None,
                      help='Thư mục lưu kết quả')
    parser.add_argument('--launch_file', default=None,
                      help='Tự động khởi động file launch (ví dụ: custom_robot/fixed_size_gmapping.launch)')
    parser.add_argument('--launch_args', default="",
                      help='Các tham số cho file launch (ví dụ: map_size:=384 map_resolution:=0.05)')
    parser.add_argument('--auto_close', action='store_true',
                      help='Tự động đóng file launch sau khi hoàn thành giám sát')
    
    args = parser.parse_args()
    
    # Biến để lưu trữ quy trình launch
    launch_process = None
    
    # Nếu được chỉ định, khởi động file launch
    if args.launch_file:
        launch_cmd = f"roslaunch {args.launch_file} {args.launch_args}"
        print(f"Khởi động: {launch_cmd}")
        
        # Khởi động file launch trong quy trình riêng biệt
        launch_process = subprocess.Popen(launch_cmd, shell=True, preexec_fn=os.setsid)
        
        # Chờ một chút để SLAM khởi động
        print("Đợi SLAM khởi động...")
        time.sleep(5)
    
    try:
        # Tạo đối tượng giám sát tài nguyên
        monitor = SLAMResourceMonitor(args.method, args.duration, args.interval, args.output_dir)
        
        # Bắt đầu giám sát
        monitor.monitor_resources()
        
    except KeyboardInterrupt:
        print("\nĐã dừng giám sát bằng tay.")
    
    finally:
        # Nếu được yêu cầu, đóng file launch
        if launch_process and args.auto_close:
            print(f"Đóng file launch...")
            os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
            launch_process.wait()

if __name__ == "__main__":
    main()