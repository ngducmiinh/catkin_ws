#!/bin/bash

# Đường dẫn đến thư mục maps
MAPS_DIR="$HOME/catkin_ws/src/custom_robot/maps"

# Tạo thư mục maps nếu nó không tồn tại
mkdir -p $MAPS_DIR

# Hàm hiển thị hướng dẫn sử dụng
usage() {
    echo "Sử dụng: $0 [options]"
    echo "  options:"
    echo "    -n, --name NAME    Tên của bản đồ (mặc định: my_map_TIMESTAMP)"
    echo "    -l, --list         Liệt kê các bản đồ đã lưu"
    echo "    -h, --help         Hiển thị hướng dẫn này"
}

# Tạo tên map mặc định với timestamp
DEFAULT_NAME="my_map_$(date +%Y%m%d_%H%M%S)"
MAP_NAME=$DEFAULT_NAME

# Xử lý các tham số
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--name)
            MAP_NAME="$2"
            shift 2
            ;;
        -l|--list)
            echo "Danh sách các bản đồ đã lưu:"
            ls -la $MAPS_DIR
            exit 0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Tùy chọn không hợp lệ: $1"
            usage
            exit 1
            ;;
    esac
done

# Đường dẫn đầy đủ đến file map
MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "Đang lưu bản đồ với tên '$MAP_NAME'..."
echo "Đường dẫn lưu: $MAP_PATH"

# Lưu bản đồ bằng map_server
rosrun map_server map_saver -f "$MAP_PATH"

# Kiểm tra kết quả
if [ $? -eq 0 ]; then
    echo "Bản đồ đã được lưu thành công!"
    echo "File PGM: ${MAP_PATH}.pgm"
    echo "File YAML: ${MAP_PATH}.yaml"
else
    echo "Lỗi khi lưu bản đồ. Hãy đảm bảo SLAM đang chạy và tạo bản đồ."
fi