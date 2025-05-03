#!/bin/bash

# Script để chạy SLAM với kích thước bản đồ cố định và lưu bản đồ
# Tác giả: AI Assistant
# Ngày: 3/5/2025

# Kiểm tra số lượng đối số
if [ "$#" -lt 1 ]; then
    echo "Cách sử dụng: $0 <slam_type> [map_size] [resolution]"
    echo "  slam_type: gmapping hoặc hector"
    echo "  map_size: Kích thước bản đồ theo pixels (mặc định: 384)"
    echo "  resolution: Độ phân giải bản đồ (mặc định: 0.05 m/pixel)"
    exit 1
fi

# Đối số
SLAM_TYPE=$1
MAP_SIZE=${2:-384}
RESOLUTION=${3:-0.05}
MAP_NAME="map_${SLAM_TYPE}_${MAP_SIZE}x${MAP_SIZE}_${RESOLUTION}"
OUTPUT_DIR=~/maps

# Tạo thư mục đầu ra nếu chưa tồn tại
mkdir -p $OUTPUT_DIR

# Kiểm tra loại SLAM
if [ "$SLAM_TYPE" != "gmapping" ] && [ "$SLAM_TYPE" != "hector" ]; then
    echo "Loại SLAM không hợp lệ. Chỉ hỗ trợ: gmapping, hector"
    exit 1
fi

echo "Khởi chạy SLAM $SLAM_TYPE với kích thước $MAP_SIZE x $MAP_SIZE pixels, độ phân giải $RESOLUTION m/pixel"

# Khởi chạy SLAM với kích thước cố định
if [ "$SLAM_TYPE" == "gmapping" ]; then
    roslaunch custom_robot fixed_size_gmapping.launch map_size:=$MAP_SIZE map_resolution:=$RESOLUTION &
else
    roslaunch custom_robot fixed_size_hector.launch map_size:=$MAP_SIZE map_resolution:=$RESOLUTION &
fi

SLAM_PID=$!

# Đợi người dùng nhấn Enter để lưu bản đồ
echo "SLAM đang chạy. Di chuyển robot để xây dựng bản đồ."
echo "Nhấn Enter để lưu bản đồ và kết thúc..."
read

# Lưu bản đồ
echo "Đang lưu bản đồ tại $OUTPUT_DIR/$MAP_NAME..."
rosrun map_server map_saver -f $OUTPUT_DIR/$MAP_NAME

# Kết thúc quá trình SLAM
echo "Đang kết thúc quá trình SLAM..."
kill $SLAM_PID

echo "Hoàn tất! Bản đồ đã được lưu tại: $OUTPUT_DIR/$MAP_NAME.pgm và $OUTPUT_DIR/$MAP_NAME.yaml"
echo "Bạn có thể so sánh chất lượng bản đồ bằng cách chạy:"
echo "rosrun custom_robot map_quality_test.py --ref-file $OUTPUT_DIR/map_gmapping_${MAP_SIZE}x${MAP_SIZE}_${RESOLUTION}.yaml --test-file $OUTPUT_DIR/map_hector_${MAP_SIZE}x${MAP_SIZE}_${RESOLUTION}.yaml"