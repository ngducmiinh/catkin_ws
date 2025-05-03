#!/bin/bash

# Script tự động chạy đánh giá SLAM
# Sử dụng: ./run_slam_evaluation.sh [gmapping|hector|both] [auto|manual]
# Ví dụ: ./run_slam_evaluation.sh gmapping auto

# Thiết lập các thư mục
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"
BAGS_DIR="$CATKIN_WS/src/custom_robot/evaluation/bags"
RESULTS_DIR="$CATKIN_WS/src/custom_robot/evaluation/results"

# Tạo thư mục nếu chưa tồn tại
mkdir -p "$BAGS_DIR" "$RESULTS_DIR"

# Xác định phương pháp SLAM cần đánh giá
METHOD="$1"
if [ -z "$METHOD" ]; then
    echo "Không có phương pháp SLAM được chỉ định. Sử dụng mặc định: both"
    METHOD="both"
fi

if [ "$METHOD" != "gmapping" ] && [ "$METHOD" != "hector" ] && [ "$METHOD" != "both" ]; then
    echo "Lỗi: Phương pháp không hợp lệ. Vui lòng chọn 'gmapping', 'hector' hoặc 'both'."
    exit 1
fi

# Xác định chế độ di chuyển (tự động hoặc thủ công)
MOVEMENT_MODE="$2"
if [ -z "$MOVEMENT_MODE" ]; then
    echo "Không có chế độ di chuyển được chỉ định. Sử dụng mặc định: auto"
    MOVEMENT_MODE="auto"
fi

if [ "$MOVEMENT_MODE" != "auto" ] && [ "$MOVEMENT_MODE" != "manual" ]; then
    echo "Lỗi: Chế độ di chuyển không hợp lệ. Vui lòng chọn 'auto' hoặc 'manual'."
    exit 1
fi

# Thời gian thu thập dữ liệu (giây)
DURATION=120

# Timestamp cho tên file
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=== ĐÁNH GIÁ SLAM BẮT ĐẦU - $(date) ==="
echo "Thư mục bags: $BAGS_DIR"
echo "Thư mục kết quả: $RESULTS_DIR"
echo "Chế độ di chuyển: $MOVEMENT_MODE"

# Tạo một thư mục cho phiên đánh giá này
SESSION_DIR="$RESULTS_DIR/session_$TIMESTAMP"
mkdir -p "$SESSION_DIR"

# Ghi log (sử dụng phương pháp đơn giản hơn để tránh lỗi)
LOG_FILE="$SESSION_DIR/evaluation_log.txt"
echo "=== ĐÁNH GIÁ SLAM - $(date) ===" > "$LOG_FILE"
echo "Thư mục bags: $BAGS_DIR" >> "$LOG_FILE"
echo "Thư mục kết quả: $RESULTS_DIR" >> "$LOG_FILE"

# Kiểm tra evo đã được cài đặt chưa
if ! command -v evo_ape &> /dev/null; then
    echo "ERROR: Gói evo chưa được cài đặt."
    echo "Vui lòng cài đặt bằng lệnh: pip install evo"
    echo "Tham khảo: https://github.com/MichaelGrupp/evo"
    echo "ERROR: Gói evo chưa được cài đặt." >> "$LOG_FILE"
    exit 1
fi

# Dừng tất cả tiến trình ROS đang chạy
function stop_ros_processes() {
    echo "Dừng tất cả tiến trình ROS đang chạy..." | tee -a "$LOG_FILE"
    
    # Dừng nhẹ nhàng trước
    rosnode kill -a &>/dev/null || true
    sleep 2
    
    # Sau đó dừng mạnh nếu cần
    # Chỉ kill các tiến trình ROS cụ thể
    pkill -f ros || true
    pkill -f slam_gmapping || true
    pkill -f hector || true
    pkill -f roslaunch || true
    pkill -f roscore || true
    pkill -f gazebo || true
    pkill -f test_movement.py || true
    pkill -f record_slam_data.py || true
    
    # Đợi mọi thứ dừng lại
    sleep 3
    
    echo "Đã dừng tất cả tiến trình ROS." | tee -a "$LOG_FILE"
}

# Xử lý khi nhấn Ctrl+C
trap cleanup INT

function cleanup() {
    echo -e "\nNhận tín hiệu ngắt. Đang dọn dẹp..." | tee -a "$LOG_FILE"
    stop_ros_processes
    echo "Thoát script đánh giá." | tee -a "$LOG_FILE"
    exit 1
}

# Kiểm tra xem một tiến trình có tồn tại và chạy không
function check_process() {
    local pid=$1
    if [ -n "$pid" ] && ps -p "$pid" &>/dev/null; then
        return 0  # Tiến trình đang chạy
    else
        return 1  # Tiến trình không tồn tại hoặc không chạy
    fi
}

# Hàm thu thập dữ liệu SLAM
function collect_data() {
    local slam_method=$1
    
    # Dừng tất cả tiến trình ROS trước khi bắt đầu
    stop_ros_processes
    
    # Khởi động roscore
    echo "Khởi động roscore..." | tee -a "$LOG_FILE"
    roscore &
    ROSCORE_PID=$!
    sleep 5
    
    # Kiểm tra roscore có chạy không
    if ! check_process $ROSCORE_PID; then
        echo "Lỗi: Không thể khởi động roscore." | tee -a "$LOG_FILE"
        return 1
    fi
    
    echo "=== THU THẬP DỮ LIỆU $slam_method ===" | tee -a "$LOG_FILE"
    
    # Khởi động mô phỏng Gazebo
    echo "Khởi động môi trường mô phỏng Gazebo..." | tee -a "$LOG_FILE"
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_world.launch &
    GAZEBO_PID=$!
    sleep 15  # Đợi Gazebo khởi động đầy đủ
    
    # Kiểm tra Gazebo có chạy không
    if ! check_process $GAZEBO_PID; then
        echo "Lỗi: Không thể khởi động Gazebo." | tee -a "$LOG_FILE"
        stop_ros_processes
        return 1
    fi
    
    # Khởi động SLAM
    echo "Khởi động $slam_method SLAM..." | tee -a "$LOG_FILE"
    roslaunch custom_robot custom_slam.launch slam_methods:=$slam_method &
    SLAM_PID=$!
    sleep 10  # Đợi SLAM khởi động
    
    # Kiểm tra SLAM có chạy không
    if ! check_process $SLAM_PID; then
        echo "Lỗi: Không thể khởi động $slam_method SLAM." | tee -a "$LOG_FILE"
        stop_ros_processes
        return 1
    fi
    
    # Bắt đầu ghi dữ liệu
    echo "Bắt đầu ghi dữ liệu rosbag cho $slam_method trong $DURATION giây..." | tee -a "$LOG_FILE"
    BAG_FILE="$BAGS_DIR/${slam_method}_$TIMESTAMP.bag"
    
    # Trực tiếp ghi dữ liệu rosbag
    echo "Ghi dữ liệu vào: $BAG_FILE" | tee -a "$LOG_FILE"
    rosbag record -O "$BAG_FILE" /scan /tf /map /odom /gazebo/model_states /tf_static __name:=rosbag_record &
    RECORD_PID=$!
    sleep 5
    
    # Di chuyển robot
    if [ "$MOVEMENT_MODE" = "auto" ]; then
        echo "Chế độ tự động: Robot sẽ di chuyển theo mẫu khám phá..." | tee -a "$LOG_FILE"
        
        # Kiểm tra script di chuyển tự động
        if [ ! -f "$SCRIPT_DIR/test_movement.py" ]; then
            # Tạo script di chuyển tự động nếu chưa có
            create_movement_script
        fi
        
        # Chạy script di chuyển tự động
        chmod +x "$SCRIPT_DIR/test_movement.py"
        python3 "$SCRIPT_DIR/test_movement.py" --duration $DURATION --exploration &
        MOVE_PID=$!
    else
        echo -e "\n=====================================" | tee -a "$LOG_FILE"
        echo "CHẾ ĐỘ THỦ CÔNG: Vui lòng điều khiển robot bằng bàn phím" | tee -a "$LOG_FILE"
        echo "Mở terminal mới và chạy lệnh sau:" | tee -a "$LOG_FILE"
        echo "  roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" | tee -a "$LOG_FILE"
        echo -e "=====================================\n" | tee -a "$LOG_FILE"
    fi
    
    # Đợi đủ thời gian thu thập dữ liệu
    echo "Đang thu thập dữ liệu... (trong $DURATION giây)" | tee -a "$LOG_FILE"
    countdown=$DURATION
    while [ $countdown -gt 0 ]; do
        if [ $((countdown % 10)) -eq 0 ]; then
            echo "Còn $countdown giây..." | tee -a "$LOG_FILE"
        fi
        sleep 1
        countdown=$((countdown - 1))
    done
    
    # Dừng ghi dữ liệu
    echo "Dừng ghi dữ liệu..." | tee -a "$LOG_FILE"
    rosnode kill /rosbag_record
    sleep 2
    
    echo "Đã hoàn thành việc thu thập dữ liệu cho $slam_method." | tee -a "$LOG_FILE"
    echo "File bag: $BAG_FILE" | tee -a "$LOG_FILE"
    
    # Dừng các tiến trình
    if [ "$MOVEMENT_MODE" = "auto" ] && check_process $MOVE_PID; then
        kill $MOVE_PID 2>/dev/null || true
    fi
    
    # Dừng các tiến trình SLAM và Gazebo
    stop_ros_processes
    
    # Kiểm tra file bag có tồn tại và không rỗng
    if [ ! -f "$BAG_FILE" ] || [ ! -s "$BAG_FILE" ]; then
        echo "Lỗi: File bag không tồn tại hoặc rỗng!" | tee -a "$LOG_FILE"
        return 1
    fi
    
    return 0
}

# Hàm đánh giá kết quả SLAM
function evaluate_slam() {
    local slam_method=$1
    local bag_file="$BAGS_DIR/${slam_method}_$TIMESTAMP.bag"
    
    echo "=== ĐÁNH GIÁ $slam_method SLAM ===" | tee -a "$LOG_FILE"
    
    if [ ! -f "$bag_file" ]; then
        echo "Lỗi: Không tìm thấy file bag cho $slam_method: $bag_file" | tee -a "$LOG_FILE"
        return 1
    fi
    
    echo "Đánh giá $slam_method sử dụng file: $bag_file" | tee -a "$LOG_FILE"
    
    # Gọi script đánh giá và ghi log
    python3 "$SCRIPT_DIR/evaluate_slam.py" --bag "$bag_file" --method "$slam_method" --output_dir "$SESSION_DIR/$slam_method" 2>&1 | tee -a "$LOG_FILE"
    
    # Lấy mã trạng thái của lệnh python (sẽ là mã trạng thái của lệnh đầu tiên trong pipe)
    local status=${PIPESTATUS[0]}
    
    if [ $status -eq 0 ]; then
        echo "Đã hoàn thành đánh giá $slam_method." | tee -a "$LOG_FILE"
        return 0
    else
        echo "Lỗi khi đánh giá $slam_method (mã lỗi: $status)." | tee -a "$LOG_FILE"
        return 1
    fi
}

# Hàm tạo script di chuyển tự động
function create_movement_script() {
    echo "Đang tạo script di chuyển tự động..." | tee -a "$LOG_FILE"
    cat > "$SCRIPT_DIR/test_movement.py" << 'EOF'
#!/usr/bin/env python3

import rospy
import sys
import time
import random
import argparse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        rospy.init_node('robot_movement_test', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(10)  # 10Hz
        self.twist = Twist()
        self.obstacle_detected = False
        self.last_position = None
        self.is_stuck = False
        self.stuck_counter = 0
        self.last_scan = None
        
    def scan_callback(self, data):
        """Xử lý dữ liệu từ LiDAR để phát hiện vật cản."""
        # Kiểm tra các khu vực phía trước robot
        min_distance = float('inf')
        ranges = data.ranges
        
        # Kiểm tra phía trước (góc từ -30 đến 30 độ)
        front_ranges = []
        front_index_min = int((0 - data.angle_min) / data.angle_increment)
        front_index_max = int((0 + data.angle_min) / data.angle_increment)
        
        for i in range(front_index_min, front_index_max + 1):
            if i < len(ranges) and i >= 0:
                if ranges[i] > 0.01:  # Loại bỏ giá trị không hợp lệ (quá nhỏ)
                    front_ranges.append(ranges[i])
        
        if front_ranges:
            min_distance = min(front_ranges)
            
        # Thiết lập ngưỡng an toàn
        safety_threshold = 0.5  # 0.5m
        
        self.obstacle_detected = min_distance < safety_threshold
        self.last_scan = data
        
    def odom_callback(self, data):
        """Xử lý dữ liệu từ odometry để phát hiện robot bị kẹt."""
        curr_pos = data.pose.pose.position
        
        if self.last_position is not None:
            # Tính khoảng cách di chuyển
            dist = ((curr_pos.x - self.last_position.x)**2 + 
                    (curr_pos.y - self.last_position.y)**2)**0.5
            
            # Nếu robot hầu như không di chuyển, tăng bộ đếm kẹt
            if dist < 0.01:  # Ngưỡng di chuyển tối thiểu
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
                
            # Xác định robot bị kẹt nếu không di chuyển sau nhiều lần thử
            self.is_stuck = self.stuck_counter > 50  # ~5 giây
            
        self.last_position = curr_pos
        
    def random_movement(self):
        """Di chuyển ngẫu nhiên nhưng có mục đích để khám phá môi trường."""
        if self.obstacle_detected or self.is_stuck:
            # Xoay để tránh chướng ngại vật hoặc thoát khỏi tình trạng kẹt
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.7  # Xoay mạnh khi gặp chướng ngại vật hoặc bị kẹt
            
            if self.is_stuck:
                print("Robot bị kẹt! Đang cố gắng giải phóng...")
                # Nếu bị kẹt quá lâu, thử lùi lại
                if self.stuck_counter > 100:
                    self.twist.linear.x = -0.1
            
            # Reset bộ đếm kẹt sau khi xử lý xoay
            if self.stuck_counter > 150:
                self.stuck_counter = 0
        else:
            # Di chuyển tiến và xoay ngẫu nhiên
            self.twist.linear.x = 0.15  # Di chuyển chậm để thu thập dữ liệu tốt hơn
            
            # Thỉnh thoảng đổi hướng để khám phá nhiều hơn
            if random.random() < 0.05:  # 5% cơ hội đổi hướng
                self.twist.angular.z = random.uniform(-0.5, 0.5)
            elif random.random() < 0.01:  # 1% cơ hội dừng và xoay
                self.twist.linear.x = 0.0
                self.twist.angular.z = random.choice([-0.7, 0.7])
                
        self.pub.publish(self.twist)
        
    def exploration_movement(self):
        """Di chuyển theo kiểu khám phá mục tiêu."""
        if not self.last_scan:
            return  # Đợi dữ liệu LiDAR
            
        # Phân tích dữ liệu LiDAR để tìm khu vực trống nhất
        ranges = self.last_scan.ranges
        sectors = 8  # Chia không gian thành 8 khu vực
        sector_ranges = [[] for _ in range(sectors)]
        
        for i, r in enumerate(ranges):
            if r > 0.01:  # Bỏ qua giá trị không hợp lệ
                sector_idx = min(int(i * sectors / len(ranges)), sectors - 1)
                sector_ranges[sector_idx].append(r)
        
        # Tính khoảng cách trung bình cho mỗi khu vực
        sector_avg = []
        for sector in sector_ranges:
            if sector:
                sector_avg.append(sum(sector) / len(sector))
            else:
                sector_avg.append(0.0)
        
        # Tìm khu vực có không gian trống lớn nhất
        max_space_idx = sector_avg.index(max(sector_avg))
        target_angle = (max_space_idx + 0.5) * 2 * 3.14159 / sectors
        
        # Tính góc để xoay đến khu vực có không gian trống
        current_angle = sectors / 2  # Giả sử robot hướng về trước
        angle_diff = target_angle - current_angle
        
        if self.obstacle_detected or self.is_stuck:
            # Xử lý khi gặp chướng ngại vật hoặc bị kẹt
            if self.is_stuck:
                print("Robot bị kẹt! Đang cố gắng giải phóng...")
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.7
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.7  # Xoay để tránh
                
            # Reset bộ đếm kẹt sau khi xử lý
            if self.stuck_counter > 150:
                self.stuck_counter = 0
        else:
            # Điều chỉnh hướng đi
            if abs(angle_diff) > 0.2:
                # Cần xoay nhiều để hướng đến khu vực trống
                self.twist.linear.x = 0.05
                self.twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Đã đúng hướng, di chuyển thẳng
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.0
                
        self.pub.publish(self.twist)
        
    def stop(self):
        """Dừng robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        
def main():
    parser = argparse.ArgumentParser(description='Di chuyển robot tự động để thử nghiệm SLAM')
    parser.add_argument('--duration', type=int, default=120,
                      help='Thời gian chạy (giây)')
    parser.add_argument('--exploration', action='store_true',
                      help='Sử dụng thuật toán khám phá thông minh')
    args = parser.parse_args()
    
    controller = RobotController()
    start_time = time.time()
    
    try:
        print(f"Bắt đầu di chuyển robot tự động trong {args.duration} giây...")
        
        while time.time() - start_time < args.duration and not rospy.is_shutdown():
            if args.exploration:
                controller.exploration_movement()
            else:
                controller.random_movement()
                
            controller.rate.sleep()
            
        print("Kết thúc thời gian di chuyển.")
        controller.stop()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.stop()
        print("Đã dừng robot.")
        
if __name__ == "__main__":
    main()
EOF
    chmod +x "$SCRIPT_DIR/test_movement.py"
    echo "Đã tạo script di chuyển tự động tại: $SCRIPT_DIR/test_movement.py" | tee -a "$LOG_FILE"
}

# --- CHƯƠNG TRÌNH CHÍNH ---

# Kiểm tra môi trường
export TURTLEBOT3_MODEL=burger

# Tạo script di chuyển tự động nếu cần
if [ "$MOVEMENT_MODE" = "auto" ] && [ ! -f "$SCRIPT_DIR/test_movement.py" ]; then
    create_movement_script
fi

# Thực hiện thu thập dữ liệu và đánh giá cho Gmapping
if [ "$METHOD" = "gmapping" ] || [ "$METHOD" = "both" ]; then
    echo "--- BẮT ĐẦU ĐÁNH GIÁ GMAPPING ---" | tee -a "$LOG_FILE"
    collect_data "gmapping"
    evaluate_slam "gmapping"
    echo "--- KẾT THÚC ĐÁNH GIÁ GMAPPING ---" | tee -a "$LOG_FILE"
fi

# Thực hiện thu thập dữ liệu và đánh giá cho Hector
if [ "$METHOD" = "hector" ] || [ "$METHOD" = "both" ]; then
    echo "--- BẮT ĐẦU ĐÁNH GIÁ HECTOR ---" | tee -a "$LOG_FILE"
    collect_data "hector"
    evaluate_slam "hector"
    echo "--- KẾT THÚC ĐÁNH GIÁ HECTOR ---" | tee -a "$LOG_FILE"
fi

# So sánh cả hai phương pháp nếu đã thu thập đủ dữ liệu
if [ "$METHOD" = "both" ]; then
    echo "=== SO SÁNH GMAPPING VÀ HECTOR SLAM ===" | tee -a "$LOG_FILE"
    
    # Kiểm tra file quỹ đạo đã tồn tại chưa
    GMAPPING_TRAJ=$(find "$SESSION_DIR/gmapping" -name "gmapping_trajectory_*.txt" | head -1)
    HECTOR_TRAJ=$(find "$SESSION_DIR/hector" -name "hector_trajectory_*.txt" | head -1)
    GT_FILE=$(find "$SESSION_DIR/gmapping" -name "groundtruth_*.txt" | head -1)
    
    if [ -z "$GMAPPING_TRAJ" ] || [ -z "$HECTOR_TRAJ" ] || [ -z "$GT_FILE" ]; then
        echo "Không tìm thấy đủ các file quỹ đạo để so sánh:" | tee -a "$LOG_FILE"
        echo "  Gmapping: $GMAPPING_TRAJ" | tee -a "$LOG_FILE"
        echo "  Hector: $HECTOR_TRAJ" | tee -a "$LOG_FILE"
        echo "  Ground truth: $GT_FILE" | tee -a "$LOG_FILE"
    else
        echo "So sánh Gmapping và Hector SLAM..." | tee -a "$LOG_FILE"
        python3 "$SCRIPT_DIR/compare_slam_methods.py" --gt "$GT_FILE" --gmapping "$GMAPPING_TRAJ" --hector "$HECTOR_TRAJ" --output_dir "$SESSION_DIR/comparison" 2>&1 | tee -a "$LOG_FILE"
    fi
fi

# Dừng tất cả tiến trình ROS còn lại
stop_ros_processes

echo "=== ĐÁNH GIÁ SLAM HOÀN THÀNH - $(date) ===" | tee -a "$LOG_FILE"
echo "Kết quả được lưu tại: $SESSION_DIR" | tee -a "$LOG_FILE"

# Hiển thị đường dẫn đến kết quả
echo ""
echo "Tóm tắt kết quả:"
echo "- Log đánh giá: $LOG_FILE"
echo "- Thư mục kết quả: $SESSION_DIR"
if [ -d "$SESSION_DIR/gmapping" ]; then
    echo "- Kết quả Gmapping: $SESSION_DIR/gmapping"
fi
if [ -d "$SESSION_DIR/hector" ]; then
    echo "- Kết quả Hector: $SESSION_DIR/hector"
fi
if [ -d "$SESSION_DIR/comparison" ]; then
    echo "- Kết quả so sánh: $SESSION_DIR/comparison"
fi