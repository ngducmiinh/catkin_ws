#!/bin/bash

# Tạo thư mục lưu rosbag nếu chưa tồn tại
mkdir -p ~/catkin_ws/bags

# Kiểm tra xem người dùng đã cung cấp tên cho file bag chưa
if [ $# -eq 0 ]; then
    echo "Usage: $0 <bag_name>"
    echo "Example: $0 slam_test"
    exit 1
fi

BAG_NAME=$1
BAG_PATH=~/catkin_ws/bags/${BAG_NAME}.bag

# Record các topic cần thiết cho đánh giá SLAM
# - /scan: dữ liệu LiDAR
# - /tf: transformations (quan trọng cho SLAM)
# - /map: bản đồ được tạo ra bởi thuật toán SLAM
# - /odom: odometry từ encoders
# - Ground truth nếu có (trong môi trường mô phỏng có thể là /gazebo/model_states)
# Có thể điều chỉnh danh sách topics tùy theo setup cụ thể

echo "Recording data to $BAG_PATH"
echo "Press Ctrl+C to stop recording"

rosbag record -O $BAG_PATH \
    /scan \
    /tf \
    /tf_static \
    /map \
    /odom \
    /ground_truth/pose \
    /ground_truth/path \
    /gmapping/trajectory \
    /hector/trajectory \
    __name:=slam_data_recording

echo "Data recorded to $BAG_PATH"