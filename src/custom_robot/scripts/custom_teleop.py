#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

MSG = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s : stop
q/e/z/c : curve movement

CTRL-C to quit
"""

# Tốc độ tối đa
MAX_LINEAR = 100  # Điều chỉnh giá trị này phù hợp với robot của bạn
MAX_ANGULAR = 50  # Điều chỉnh giá trị này phù hợp với robot của bạn

# Tốc độ giảm dần
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 5

class CustomTeleop:
    def __init__(self):
        rospy.init_node('custom_teleop')
        self.pub = rospy.Publisher('robot_control', Float32MultiArray, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.linear_vel = 0
        self.angular_vel = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, linear, angular):
        return "currently:\tlinear vel %s\t angular vel %s" % (linear, angular)

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        return input

    def publish_cmd(self, left_val, right_val):
        """Xuất bản tín hiệu điều khiển dạng Float32MultiArray"""
        msg = Float32MultiArray()
        msg.data = [left_val, right_val]
        self.pub.publish(msg)

    def run(self):
        try:
            print(MSG)
            print(self.vels(self.linear_vel, self.angular_vel))

            while not rospy.is_shutdown():
                key = self.getKey()
                
                # Xử lý phím bấm
                if key == 'w':
                    self.linear_vel += LIN_VEL_STEP_SIZE
                    self.linear_vel = self.constrain(self.linear_vel, -MAX_LINEAR, MAX_LINEAR)
                    print(self.vels(self.linear_vel, self.angular_vel))
                elif key == 'x':
                    self.linear_vel -= LIN_VEL_STEP_SIZE
                    self.linear_vel = self.constrain(self.linear_vel, -MAX_LINEAR, MAX_LINEAR)
                    print(self.vels(self.linear_vel, self.angular_vel))
                elif key == 'a':
                    self.angular_vel += ANG_VEL_STEP_SIZE
                    self.angular_vel = self.constrain(self.angular_vel, -MAX_ANGULAR, MAX_ANGULAR)
                    print(self.vels(self.linear_vel, self.angular_vel))
                elif key == 'd':
                    self.angular_vel -= ANG_VEL_STEP_SIZE
                    self.angular_vel = self.constrain(self.angular_vel, -MAX_ANGULAR, MAX_ANGULAR)
                    print(self.vels(self.linear_vel, self.angular_vel))
                elif key == 's':
                    self.linear_vel = 0
                    self.angular_vel = 0
                    print(self.vels(self.linear_vel, self.angular_vel))
                elif key == 'q':
                    # Cua trái
                    left = self.linear_vel - self.angular_vel
                    right = self.linear_vel + self.angular_vel
                    self.publish_cmd(left, right)
                    continue
                elif key == 'e':
                    # Cua phải
                    left = self.linear_vel + self.angular_vel
                    right = self.linear_vel - self.angular_vel
                    self.publish_cmd(left, right)
                    continue
                elif key == 'z':
                    # Xoay trái
                    left = -self.linear_vel
                    right = self.linear_vel
                    self.publish_cmd(left, right)
                    continue
                elif key == 'c':
                    # Xoay phải
                    left = self.linear_vel
                    right = -self.linear_vel
                    self.publish_cmd(left, right)
                    continue
                elif key == '\x03':  # CTRL+C
                    break
                
                # Tính toán giá trị cho bánh trái và phải
                left = self.linear_vel - self.angular_vel
                right = self.linear_vel + self.angular_vel
                
                # Xuất bản giá trị cho robot
                self.publish_cmd(left, right)

        except Exception as e:
            print(e)

        finally:
            # Dừng robot khi kết thúc
            stop_msg = Float32MultiArray()
            stop_msg.data = [0, 0]
            self.pub.publish(stop_msg)
            
            # Khôi phục thiết lập terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    teleop = CustomTeleop()
    teleop.run()