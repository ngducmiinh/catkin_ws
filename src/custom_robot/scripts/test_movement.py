#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import time

def test_movement():
    rospy.init_node('test_movement')
    pub = rospy.Publisher('robot_control', Float32MultiArray, queue_size=10)
    
    # Đợi publisher được khởi tạo
    time.sleep(1)
    
    # Tạo tin nhắn
    msg = Float32MultiArray()
    
    # Thử nghiệm các chuyển động cơ bản
    movements = [
        {"name": "Dừng", "left": 0, "right": 0, "duration": 2},
        {"name": "Tiến", "left": 100, "right": 100, "duration": 3},
        {"name": "Dừng", "left": 0, "right": 0, "duration": 2},
        {"name": "Lùi", "left": -100, "right": -100, "duration": 3},
        {"name": "Dừng", "left": 0, "right": 0, "duration": 2},
        {"name": "Rẽ trái", "left": 0, "right": 100, "duration": 3},
        {"name": "Dừng", "left": 0, "right": 0, "duration": 2},
        {"name": "Rẽ phải", "left": 100, "right": 0, "duration": 3},
        {"name": "Dừng", "left": 0, "right": 0, "duration": 2},
    ]
    
    for move in movements:
        print(f"Thực hiện: {move['name']}")
        msg.data = [move['left'], move['right']]
        pub.publish(msg)
        time.sleep(move['duration'])
    
    print("Hoàn thành kiểm tra chuyển động!")

if __name__ == '__main__':
    try:
        test_movement()
    except rospy.ROSInterruptException:
        pass