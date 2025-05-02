#!/usr/bin/env python3

import rospy
import sys
import select
import os
from std_msgs.msg import Float32MultiArray
import threading

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

LINEAR_SPEED_PWM = 120
ANGULAR_SPEED_PWM = 80

MSG = """
Control Your Robot!
-------------------

Reading from the keyboard and Publishing to robot_control!

Moving around:
    i    
j    k    l
    ,    

i: tiến
j: xoay trái
l: xoay phải
,: lùi
k: dừng

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 1),
    'j': (-1, 1),
    'l': (1, -1),
    ',': (-1, -1),
    'k': (0, 0)
}

class TeleopKey(threading.Thread):
    def __init__(self, rate):
        super(TeleopKey, self).__init__()

        self.pub = rospy.Publisher('robot_control', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(10)  # Publish rate (Hz)
        self.left_pwm = 0
        self.right_pwm = 0
        self.condition = threading.Condition()
        self.done = False
        self.speed = 0
        self.current_key = 'k'

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.pub.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
    
    def update(self, left_dir, right_dir, linear_speed, angular_speed, key):
        self.condition.acquire()
        # Lưu hướng điều khiển
        self.left_dir = left_dir
        self.right_dir = right_dir
        
        # Lưu lại phím được nhấn
        if key:
            self.current_key = key
            
        # Thiết lập tốc độ dựa vào phím
        if self.current_key == 'i' or self.current_key == ',':
            self.speed = linear_speed
        elif self.current_key == 'j' or self.current_key == 'l':
            self.speed = angular_speed
        elif self.current_key == 'k':
            self.speed = 0
        
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 'k')
        self.join()

    def run(self):
        pwm_msg = Float32MultiArray()
        
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Tính toán PWM cho mỗi bánh xe
            left_pwm = self.left_dir * self.speed
            right_pwm = self.right_dir * self.speed
            
            self.condition.release()
            
            # Cập nhật thông tin tốc độ hiện tại
            if self.speed != 0:
                print(vels(left_pwm, right_pwm))
                
            # Chuẩn bị dữ liệu để publish
            pwm_msg.data = [left_pwm, right_pwm]
            
            # Publish.
            self.pub.publish(pwm_msg)
            self.rate.sleep()

        # Publish stop message when thread exits.
        pwm_msg.data = [0.0, 0.0]
        self.pub.publish(pwm_msg)


def getKey(key_timeout):
    if os.name == 'nt':
        if key_timeout is None:
            return msvcrt.getch().decode('utf-8')
        else:
            start_time = time.time()
            while True:
                if msvcrt.kbhit():
                    return msvcrt.getch().decode('utf-8')
                if time.time() - start_time > key_timeout:
                    return ''
    else:
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def vels(left, right):
    return "Tốc độ hiện tại:\tBánh trái: %s\tBánh phải: %s " % (left, right)


if __name__ == '__main__':
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    try:
        rospy.init_node('teleop_keyboard')

        linear_speed = rospy.get_param("~linear_speed", LINEAR_SPEED_PWM)
        angular_speed = rospy.get_param("~angular_speed", ANGULAR_SPEED_PWM)
        repeat = rospy.get_param("~repeat_rate", 0.0)
        key_timeout = rospy.get_param("~key_timeout", 0.0)
        if key_timeout == 0.0:
            key_timeout = None

        pub_thread = TeleopKey(repeat)

        left_dir = 0
        right_dir = 0
        
        print("Tốc độ tuyến tính: %s, Tốc độ góc: %s" % (linear_speed, angular_speed))
        print(MSG)
        
        pub_thread.wait_for_subscribers()
        
        while True:
            key = getKey(key_timeout)
            
            if key in moveBindings.keys():
                left_dir = moveBindings[key][0]
                right_dir = moveBindings[key][1]
                pub_thread.update(left_dir, right_dir, linear_speed, angular_speed, key)
            elif key == '\x03':  # CTRL-C
                break
            else:
                # Nếu nhận phím không hợp lệ, giữ nguyên trạng thái và tốc độ hiện tại
                continue

    except Exception as e:
        print(e)

    finally:
        # Dừng thread xuất bản
        if 'pub_thread' in locals():
            pub_thread.stop()
            
        # Khôi phục thiết lập terminal
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
