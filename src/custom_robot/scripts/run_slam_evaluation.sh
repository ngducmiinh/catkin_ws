#!/bin/bash

# Script tự động chạy đánh giá SLAM
# Sử dụng: ./run_slam_evaluation.sh [gmapping|hector|both]

# Kiểm tra evo đã được cài đặt chưa
if ! command -v evo_ape &> /dev/null; then
    echo "ERROR: Gói evo chưa được cài đặt."
    echo "Vui lòng cài đặt bằng lệnh: pip install evo"
    exit 1
fi

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

# Thời gian thu thập dữ liệu (giây)
DURATION=120

# Timestamp cho tên file
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=== ĐÁNH GIÁ SLAM BẮT ĐẦU - $(date) ==="
echo "Thư mục bags: $BAGS_DIR"
echo "Thư mục kết quả: $RESULTS_DIR"

# Tạo một thư mục cho phiên đánh giá này
SESSION_DIR="$RESULTS_DIR/session_$TIMESTAMP"
mkdir -p "$SESSION_DIR"

# Ghi log
exec > >(tee -i "$SESSION_DIR/evaluation_log.txt") 2>&1

# Hàm thu thập dữ liệu SLAM
collect_data() {
    local slam_method=$1
    
    # Kiểm tra có roscore đang chạy không
    if ! pgrep roscore > /dev/null; then
        echo "Khởi động roscore..."
        roscore &
        # Đợi roscore khởi động
        sleep 5
    fi
    
    echo "=== THU THẬP DỮ LIỆU $slam_method ==="
    
    # Khởi động SLAM
    echo "Khởi động $slam_method SLAM..."
    roslaunch custom_robot custom_slam.launch slam_methods:=$slam_method &
    SLAM_PID=$!
    
    # Đợi SLAM khởi động hoàn tất
    sleep 5
    
    # Khởi động teleop để điều khiển robot
    echo "Khởi động teleop..."
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch &
    TELEOP_PID=$!
    
    # Bắt đầu ghi dữ liệu
    echo "Bắt đầu ghi dữ liệu rosbag cho $slam_method trong $DURATION giây..."
    BAG_FILE="$BAGS_DIR/${slam_method}_$TIMESTAMP.bag"
    python3 "$SCRIPT_DIR/record_slam_data.py" --method $slam_method --duration $DURATION --name "${slam_method}_$TIMESTAMP"
    
    echo "Đã hoàn thành việc thu thập dữ liệu cho $slam_method."
    echo "File bag: $BAG_FILE"
    
    # Dừng các tiến trình
    echo "Dừng các tiến trình..."
    if ps -p $TELEOP_PID > /dev/null; then
        kill $TELEOP_PID
    fi
    
    if ps -p $SLAM_PID > /dev/null; then
        kill $SLAM_PID
    fi
    
    # Đợi mọi thứ dừng lại
    sleep 3
    
    return 0
}

# Hàm đánh giá kết quả SLAM
evaluate_slam() {
    local slam_method=$1
    local bag_file="$BAGS_DIR/${slam_method}_$TIMESTAMP.bag"
    
    echo "=== ĐÁNH GIÁ $slam_method SLAM ==="
    
    if [ ! -f "$bag_file" ]; then
        echo "Lỗi: Không tìm thấy file bag cho $slam_method: $bag_file"
        return 1
    fi
    
    echo "Đánh giá $slam_method sử dụng file: $bag_file"
    python3 "$SCRIPT_DIR/evaluate_slam.py" --bag "$bag_file" --method "$slam_method" --output_dir "$SESSION_DIR/$slam_method"
    
    echo "Đã hoàn thành đánh giá $slam_method."
    return 0
}

# Thực hiện thu thập dữ liệu và đánh giá
if [ "$METHOD" = "gmapping" ] || [ "$METHOD" = "both" ]; then
    collect_data "gmapping"
    evaluate_slam "gmapping"
fi

if [ "$METHOD" = "hector" ] || [ "$METHOD" = "both" ]; then
    collect_data "hector"
    evaluate_slam "hector"
fi

# So sánh cả hai phương pháp nếu đã thu thập đủ dữ liệu
if [ "$METHOD" = "both" ]; then
    echo "=== SO SÁNH GMAPPING VÀ HECTOR SLAM ==="
    
    # Kiểm tra file quỹ đạo đã tồn tại chưa
    GMAPPING_TRAJ=$(find "$SESSION_DIR/gmapping" -name "gmapping_trajectory_*.txt" | head -1)
    HECTOR_TRAJ=$(find "$SESSION_DIR/hector" -name "hector_trajectory_*.txt" | head -1)
    GT_FILE=$(find "$SESSION_DIR/gmapping" -name "groundtruth_*.txt" | head -1)
    
    if [ -z "$GMAPPING_TRAJ" ] || [ -z "$HECTOR_TRAJ" ] || [ -z "$GT_FILE" ]; then
        echo "Không tìm thấy đủ các file quỹ đạo để so sánh."
        echo "  Gmapping: $GMAPPING_TRAJ"
        echo "  Hector: $HECTOR_TRAJ"
        echo "  Ground truth: $GT_FILE"
    else
        echo "So sánh Gmapping và Hector SLAM..."
        python3 "$SCRIPT_DIR/compare_slam_methods.py" --gt "$GT_FILE" --gmapping "$GMAPPING_TRAJ" --hector "$HECTOR_TRAJ" --output_dir "$SESSION_DIR/comparison"
    fi
fi

echo "=== ĐÁNH GIÁ SLAM HOÀN THÀNH - $(date) ==="
echo "Kết quả được lưu tại: $SESSION_DIR"