import rospy
import sys
import select
import os
from std_msgs.msg import Float32MultiArray
import threading
import sys, select, termios, tty
import random
import time
import argparse

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

LINEAR_SPEED_PWM = 100
ANGULAR_SPEED_PWM = 90

MSG = """
Control Your Robot!
---------------------------
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
       i    
   j    k    l
       ,    

CTRL-C to quit
"""

ERROR_MSG = """
Teleop Class Failed
"""
moveBindings = {
        'i':(1,1),
        'j':(-1,1),
        'l':(1,-1),
        ',':(-1,-1),
        'k':(0,0)
    }

class TeleopKey(threading.Thread):
    def __init__(self, rate):
        super(TeleopKey, self).__init__()

        self.pub = rospy.Publisher('robot_control', Float32MultiArray, queue_size=10)
        # self.pwm_msg = Float32MultiArray()
        self.rate = rospy.Rate(0.5)  # Publish rate (Hz)
        self.left_pwm = 0
        self.right_pwm = 0
        self.condition = threading.Condition()
        self.done = False
        self.speed = 0

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
        
    def update(self, left_pwm, right_pwm, linear_speed, angular_speed, key):
        self.condition.acquire()
        self.left_pwm=left_pwm
        self.right_pwm=right_pwm
        if key == 'i' or key == ',':
            self.speed = linear_speed
        elif key == 'j' or key == 'l':
            self.speed= angular_speed
        
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0,0,0,0,'')
        self.join()

    def run(self):
        pwm_msg = Float32MultiArray()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            self.left_pwm=left_pwm*self.speed
            self.right_pwm=right_pwm*self.speed
            print(vels(self.left_pwm,self.right_pwm))
            self.condition.release()
            pwm_msg.data=[self.left_pwm,self.right_pwm]
            # Publish.
            self.pub.publish(pwm_msg)

        # Publish stop message when thread exits.
        pwm_msg.data=[0.0,0.0]
        self.pub.publish(pwm_msg)
    
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(left,right):
    return "currently:\tspeed %s\tturn %s " % (left,right)

def random_movement(duration, linear_speed, angular_speed):
    """
    Di chuyển robot ngẫu nhiên trong khoảng thời gian nhất định
    :param duration: Thời gian di chuyển (giây)
    :param linear_speed: Tốc độ tiến
    :param angular_speed: Tốc độ quay
    """
    rospy.init_node('teleop_random', anonymous=True)
    pub = rospy.Publisher('robot_control', Float32MultiArray, queue_size=10)
    
    # Đợi cho subscriber kết nối
    rospy.sleep(1.0)
    
    # Thời gian bắt đầu
    start_time = time.time()
    print(f"Bắt đầu di chuyển ngẫu nhiên trong {duration} giây...")
    
    # Các lựa chọn di chuyển
    moves = ['i', 'j', 'l', ',', 'k']
    weights = [0.4, 0.2, 0.2, 0.1, 0.1]  # Tỷ lệ xuất hiện của các lệnh di chuyển
    
    # Thời gian chạy cho mỗi lệnh
    cmd_duration_min = 0.5  # Thời gian tối thiểu cho mỗi lệnh (giây)
    cmd_duration_max = 2.0  # Thời gian tối đa cho mỗi lệnh (giây)
    
    pwm_msg = Float32MultiArray()
    
    try:
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            # Chọn lệnh di chuyển ngẫu nhiên
            move = random.choices(moves, weights=weights)[0]
            
            # Thời gian chạy lệnh hiện tại
            current_cmd_duration = random.uniform(cmd_duration_min, cmd_duration_max)
            
            # Tính toán PWM cho lệnh
            if move in moveBindings:
                left_pwm = moveBindings[move][0]
                right_pwm = moveBindings[move][1]
                
                # Áp dụng tốc độ
                if move == 'i' or move == ',':
                    speed = linear_speed
                elif move == 'j' or move == 'l':
                    speed = angular_speed
                else:
                    speed = 0
                
                # Tính toán giá trị PWM cuối cùng
                final_left_pwm = left_pwm * speed
                final_right_pwm = right_pwm * speed
                
                # Gửi lệnh di chuyển
                pwm_msg.data = [final_left_pwm, final_right_pwm]
                pub.publish(pwm_msg)
                
                print(f"Di chuyển: {move}, PWM trái: {final_left_pwm}, PWM phải: {final_right_pwm}, Thời gian: {current_cmd_duration}s")
                
                # Đợi trong khoảng thời gian của lệnh hiện tại
                time.sleep(current_cmd_duration)
            
            # Kiểm tra xem đã hết thời gian tổng cộng chưa
            if (time.time() - start_time) >= duration:
                break
        
    except Exception as e:
        print(f"Lỗi khi di chuyển ngẫu nhiên: {e}")
    finally:
        # Dừng robot
        pwm_msg.data = [0.0, 0.0]
        pub.publish(pwm_msg)
        print("Đã hoàn thành di chuyển ngẫu nhiên.")
   
if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Teleop for robot control')
    parser.add_argument('--random', action='store_true', help='Enable random movement')
    parser.add_argument('--duration', type=int, default=60, help='Duration of random movement in seconds')
    parser.add_argument('--linear_speed', type=float, default=LINEAR_SPEED_PWM, help='Linear speed PWM value')
    parser.add_argument('--angular_speed', type=float, default=ANGULAR_SPEED_PWM, help='Angular speed PWM value')
    
    args, unknown = parser.parse_known_args()
    
    # Nếu chọn chế độ di chuyển ngẫu nhiên
    if args.random:
        try:
            random_movement(args.duration, args.linear_speed, args.angular_speed)
        except rospy.ROSInterruptException:
            pass
        sys.exit(0)
    
    # Chế độ điều khiển bằng bàn phím
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    linear_speed = rospy.get_param("~linear_speed", args.linear_speed)
    angular_speed = rospy.get_param("~angular_speed", args.angular_speed)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = TeleopKey(repeat)

    left_pwm = 0
    right_pwm = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        key = ''
        pub_thread.update(left_pwm,right_pwm,linear_speed, angular_speed, key)

        print(MSG)
        
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                left_pwm = moveBindings[key][0]
                right_pwm = moveBindings[key][1]
            
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and left_pwm == 0 and right_pwm == 0:
                    continue
                left_pwm=0
                right_pwm=0
                if (key == '\x03'):
                    break
            
            pub_thread.update(left_pwm, right_pwm, linear_speed, angular_speed, key)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
