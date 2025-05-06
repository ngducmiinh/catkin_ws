
import rospy
import sys
import select
import os
from std_msgs.msg import Float32MultiArray
import threading
import sys, select, termios, tty

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

LINEAR_SPEED_PWM = 120
ANGULAR_SPEED_PWM = 60

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

   
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    linear_speed = rospy.get_param("~linear_speed", LINEAR_SPEED_PWM)
    angular_speed = rospy.get_param("~angular_speed", ANGULAR_SPEED_PWM)
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
