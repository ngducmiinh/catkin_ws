#!/bin/bash

# Script để tự động hóa quá trình kiểm thử ATE và RPE trong môi trường mô phỏng
# Tác giả: AI Assistant
# Ngày: 11/05/2025

# Mặc định các tham số
SLAM_METHOD="gmapping"
DURATION=120
WORLD="turtlebot3_world"
OUTPUT_DIR="$HOME/catkin_ws/src/custom_robot/evaluation/results/$(date +%Y%m%d_%H%M%S)"

# Hiển thị hướng dẫn sử dụng
show_usage() {
    echo "Cách sử dụng: $0 [các tùy chọn]"
    echo ""
    echo "Các tùy chọn:"
    echo "  -m, --method METHOD     Phương pháp SLAM (gmapping hoặc hector) [mặc định: gmapping]"
    echo "  -d, --duration SECONDS  Thời gian chạy mô phỏng [mặc định: 120 giây]"
    echo "  -w, --world NAME        File world cho Gazebo [mặc định: turtlebot3_world]"
    echo "  -o, --output DIR        Thư mục lưu kết quả [mặc định: ~/catkin_ws/.../TIMESTAMP]"
    echo "  -h, --help              Hiển thị hướng dẫn này"
    echo ""
    echo "Ví dụ:"
    echo "  $0 --method hector --duration 180 --world turtlebot3_house"
    exit 0
}

# Xử lý các tham số đầu vào
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--method)
            SLAM_METHOD="$2"
            if [[ "$SLAM_METHOD" != "gmapping" && "$SLAM_METHOD" != "hector" ]]; then
                echo "Lỗi: Phương pháp SLAM không hợp lệ. Chỉ hỗ trợ: gmapping, hector"
                exit 1
            fi
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -w|--world)
            WORLD="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            ;;
        *)
            echo "Lỗi: Tham số không hợp lệ: $1"
            show_usage
            ;;
    esac
done

# Tạo thư mục lưu kết quả
mkdir -p "$OUTPUT_DIR"
echo "=== ĐÁNH GIÁ SLAM ($SLAM_METHOD) ==="
echo "Thời gian chạy: $DURATION giây"
echo "Thế giới mô phỏng: $WORLD"
echo "Kết quả sẽ được lưu vào: $OUTPUT_DIR"
echo ""

# Thư mục chứa bag file
BAG_DIR="$OUTPUT_DIR/bags"
mkdir -p "$BAG_DIR"
BAG_FILE="$BAG_DIR/${SLAM_METHOD}_evaluation.bag"

echo "=== BƯỚC 1: KHỞI ĐỘNG MÔI TRƯỜNG MÔ PHỎNG ==="
# Export biến môi trường cho Turtlebot
export TURTLEBOT3_MODEL=burger

# Khởi động Gazebo với world đã chỉ định
echo "Khởi động Gazebo với world: $WORLD..."
roslaunch turtlebot3_gazebo turtlebot3_world.launch world_name:=$WORLD &
GAZEBO_PID=$!

# Đợi Gazebo khởi động
sleep 10
echo "Gazebo đã khởi động."

echo "=== BƯỚC 2: KHỞI ĐỘNG SLAM ==="
echo "Khởi động SLAM với phương pháp: $SLAM_METHOD..."
roslaunch custom_robot robot_slam.launch method:=$SLAM_METHOD &
SLAM_PID=$!

# Đợi SLAM khởi động
sleep 5
echo "SLAM đã khởi động."

echo "=== BƯỚC 3: BẮT ĐẦU GHI DỮ LIỆU ==="
echo "Bắt đầu ghi dữ liệu vào: $BAG_FILE"
# Ghi các topic cần thiết cho đánh giá
TOPICS="/scan /tf /tf_static /odom /gazebo/model_states /${SLAM_METHOD}_slam/map /${SLAM_METHOD}_slam/pose"
rosbag record -O $BAG_FILE $TOPICS &
ROSBAG_PID=$!

# Đợi rosbag khởi động
sleep 2
echo "Đang ghi rosbag..."

echo "=== BƯỚC 4: DI CHUYỂN ROBOT ==="
echo "Bắt đầu di chuyển robot ngẫu nhiên trong $DURATION giây..."
# Di chuyển robot ngẫu nhiên để thu thập dữ liệu
rosrun custom_robot custom_teleop.py --random --duration $DURATION

# Đợi thêm một chút để chắc chắn đã ghi đủ dữ liệu
sleep 5

echo "=== BƯỚC 5: DỪNG GHI DỮ LIỆU ==="
echo "Dừng ghi rosbag..."
kill -INT $ROSBAG_PID
sleep 3

echo "=== BƯỚC 6: ĐÓNG CÁC QUÁ TRÌNH ==="
echo "Đóng SLAM và Gazebo..."
kill -INT $SLAM_PID
kill -INT $GAZEBO_PID
sleep 5

echo "=== BƯỚC 7: ĐÁNH GIÁ ATE VÀ RPE ==="
echo "Bắt đầu đánh giá ATE và RPE từ dữ liệu đã ghi..."
python3 "$HOME/catkin_ws/src/custom_robot/scripts/evaluate_slam.py" --bag "$BAG_FILE" --method "$SLAM_METHOD" --output_dir "$OUTPUT_DIR"

echo "=== BƯỚC 8: GIÁM SÁT TÀI NGUYÊN (TÙY CHỌN) ==="
echo "Để giám sát tài nguyên của phương pháp SLAM, bạn có thể chạy lệnh sau:"
echo "rosrun custom_robot monitor_slam_resources.py --method $SLAM_METHOD --duration $DURATION"

echo "=== HOÀN THÀNH ==="
echo "Đánh giá SLAM đã hoàn tất. Kết quả được lưu trong: $OUTPUT_DIR"
echo "Để so sánh với phương pháp SLAM khác, hãy chạy lại script với phương pháp khác."
