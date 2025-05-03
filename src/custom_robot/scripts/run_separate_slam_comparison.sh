#!/bin/bash

# Script để so sánh Gmapping với ground truth và Hector SLAM với ground truth
# Author: GitHub Copilot
# Date: May 3, 2025

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(cd $SCRIPT_DIR/../../.. && pwd)"
BAGS_DIR="$WORKSPACE_DIR/bags"
RESULTS_DIR="$WORKSPACE_DIR/slam_results"

# Tạo thư mục lưu trữ nếu chưa tồn tại
mkdir -p $BAGS_DIR
mkdir -p $RESULTS_DIR

# Tham số thời gian chạy mặc định (giây)
DURATION=300

# Xử lý tham số
if [ "$#" -gt 0 ]; then
    DURATION=$1
fi

echo "========================================================"
echo "     SO SÁNH GMAPPING & HECTOR SLAM VỚI GROUND TRUTH"
echo "========================================================"
echo "Thời gian chạy mỗi thuật toán: ${DURATION}s"
echo "Dữ liệu sẽ được lưu trong: ${BAGS_DIR}"
echo "Kết quả phân tích sẽ được lưu trong: ${RESULTS_DIR}"
echo

echo "========================================================"
echo "GIAI ĐOẠN 1: CHẠY VÀ THU THẬP DỮ LIỆU CHO GMAPPING"
echo "========================================================"
echo "Khởi động Gmapping SLAM..."
echo "Lưu ý: Hãy điều khiển robot trong suốt thời gian này"

# Khởi động roscore nếu chưa chạy
if ! pgrep -x "rosmaster" > /dev/null; then
    echo "Khởi động roscore..."
    roscore &
    sleep 5
fi

# Đường dẫn file bag cho Gmapping
GMAPPING_BAG="$BAGS_DIR/gmapping_slam_data.bag"

# Khởi động Gmapping
echo "Khởi động Gmapping và thu thập dữ liệu trong ${DURATION} giây..."
roslaunch custom_robot fixed_size_gmapping.launch open_rviz:=true &
GMAPPING_PID=$!
sleep 5  # Đợi SLAM khởi động

# Bắt đầu thu thập dữ liệu
echo "Bắt đầu thu thập dữ liệu Gmapping..."
rosbag record -O $GMAPPING_BAG \
    /scan \
    /tf \
    /tf_static \
    /map \
    /odom \
    /ground_truth/path \
    /gmapping/trajectory \
    __name:=gmapping_recording &
ROSBAG_PID=$!

# Khởi động trajectory publisher
rosrun custom_robot publish_trajectories.py _gt_frame:=map _slam_frame:=map _robot_frame:=base_footprint _out_topic:=/ground_truth/path _slam_topic:=/gmapping/trajectory &
TRAJ_PID=$!

# Đếm ngược thời gian
echo "Thu thập dữ liệu trong ${DURATION} giây..."
for i in $(seq $DURATION -1 1); do
    echo -ne "Thời gian còn lại: ${i}s\r"
    sleep 1
done
echo -e "\nHoàn thành thu thập dữ liệu Gmapping!"

# Dừng quá trình ghi dữ liệu
rosnode kill gmapping_recording
kill $TRAJ_PID
kill $GMAPPING_PID
sleep 5

echo "========================================================"
echo "GIAI ĐOẠN 2: CHẠY VÀ THU THẬP DỮ LIỆU CHO HECTOR SLAM"
echo "========================================================"
echo "Khởi động Hector SLAM..."
echo "Lưu ý: Hãy điều khiển robot với cùng một quỹ đạo như với Gmapping"

# Đường dẫn file bag cho Hector
HECTOR_BAG="$BAGS_DIR/hector_slam_data.bag"

# Khởi động Hector SLAM
echo "Khởi động Hector SLAM và thu thập dữ liệu trong ${DURATION} giây..."
roslaunch custom_robot fixed_size_hector.launch open_rviz:=true &
HECTOR_PID=$!
sleep 5  # Đợi SLAM khởi động

# Bắt đầu thu thập dữ liệu
echo "Bắt đầu thu thập dữ liệu Hector..."
rosbag record -O $HECTOR_BAG \
    /scan \
    /tf \
    /tf_static \
    /map \
    /odom \
    /ground_truth/path \
    /hector/trajectory \
    __name:=hector_recording &
ROSBAG_PID=$!

# Khởi động trajectory publisher
rosrun custom_robot publish_trajectories.py _gt_frame:=map _slam_frame:=map _robot_frame:=base_footprint _out_topic:=/ground_truth/path _slam_topic:=/hector/trajectory &
TRAJ_PID=$!

# Đếm ngược thời gian
echo "Thu thập dữ liệu trong ${DURATION} giây..."
for i in $(seq $DURATION -1 1); do
    echo -ne "Thời gian còn lại: ${i}s\r"
    sleep 1
done
echo -e "\nHoàn thành thu thập dữ liệu Hector!"

# Dừng quá trình ghi dữ liệu
rosnode kill hector_recording
kill $TRAJ_PID
kill $HECTOR_PID
sleep 5

echo "========================================================"
echo "GIAI ĐOẠN 3: PHÂN TÍCH HIỆU SUẤT GMAPPING"
echo "========================================================"

# Phân tích dữ liệu Gmapping
echo "Phân tích dữ liệu Gmapping so với ground truth..."
python $SCRIPT_DIR/compare_slam.py $GMAPPING_BAG \
    --gt-topic /ground_truth/path \
    --slam-topic /gmapping/trajectory \
    --slam-name Gmapping \
    --output $RESULTS_DIR/gmapping_results.yaml \
    --plot-output $RESULTS_DIR/gmapping_plots.png \
    --only-one

echo "========================================================"
echo "GIAI ĐOẠN 4: PHÂN TÍCH HIỆU SUẤT HECTOR SLAM"
echo "========================================================"

# Phân tích dữ liệu Hector
echo "Phân tích dữ liệu Hector SLAM so với ground truth..."
python $SCRIPT_DIR/compare_slam.py $HECTOR_BAG \
    --gt-topic /ground_truth/path \
    --slam-topic /hector/trajectory \
    --slam-name Hector \
    --output $RESULTS_DIR/hector_results.yaml \
    --plot-output $RESULTS_DIR/hector_plots.png \
    --only-one

echo "========================================================"
echo "GIAI ĐOẠN 5: TẠO BÁO CÁO TỔNG HỢP"
echo "========================================================"

# Tạo báo cáo tổng hợp
echo "Tạo báo cáo tổng hợp..."
python $SCRIPT_DIR/generate_slam_report.py \
    --gmapping $RESULTS_DIR/gmapping_results.yaml \
    --hector $RESULTS_DIR/hector_results.yaml \
    --output $RESULTS_DIR/slam_comparison_report.html \
    --plot $RESULTS_DIR/slam_comparison_plots.png

echo "========================================================"
echo "Quá trình phân tích đã hoàn tất!"
echo "Kết quả chi tiết đã được lưu trong: $RESULTS_DIR"
echo "  - Kết quả Gmapping: $RESULTS_DIR/gmapping_results.yaml"
echo "  - Biểu đồ Gmapping: $RESULTS_DIR/gmapping_plots.png"
echo "  - Kết quả Hector: $RESULTS_DIR/hector_results.yaml"
echo "  - Biểu đồ Hector: $RESULTS_DIR/hector_plots.png"
echo "  - Báo cáo tổng hợp: $RESULTS_DIR/slam_comparison_report.html"
echo "========================================================"