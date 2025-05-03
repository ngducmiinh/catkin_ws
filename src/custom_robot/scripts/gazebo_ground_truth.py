#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from gazebo_msgs.msg import ModelStates
import sys

class GazeboGroundTruth:
    """
    Node xuất ground truth từ Gazebo thành các topic định dạng chuẩn
    để sử dụng khi đánh giá SLAM
    """
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('gazebo_ground_truth')
        
        # Lấy tên robot từ tham số (mặc định là 'turtlebot3')
        self.robot_name = rospy.get_param('~robot_name', 'turtlebot3')
        self.output_frame = rospy.get_param('~output_frame', 'map')
        
        # Tạo publisher cho các dữ liệu ground truth
        self.path_pub = rospy.Publisher('/ground_truth/path', Path, queue_size=1)
        self.pose_pub = rospy.Publisher('/ground_truth/pose', PoseStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('/ground_truth/odom', Odometry, queue_size=1)
        
        # Đăng ký theo dõi dữ liệu vị trí từ Gazebo
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Khởi tạo biến lưu quỹ đạo
        self.path = Path()
        self.path.header.frame_id = self.output_frame
        
        # Đăng ký TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo(f"Gazebo ground truth node khởi động. Theo dõi robot: {self.robot_name}")
        
    def model_states_callback(self, msg):
        """
        Xử lý dữ liệu vị trí từ Gazebo và xuất ra các topic ground truth
        """
        try:
            # Tìm chỉ mục của robot trong danh sách các model
            if self.robot_name in msg.name:
                idx = msg.name.index(self.robot_name)
            else:
                # Tìm kiếm mô hình có tên gần giống
                for i, name in enumerate(msg.name):
                    if self.robot_name.lower() in name.lower():
                        idx = i
                        self.robot_name = name  # Cập nhật để khớp chính xác
                        rospy.loginfo(f"Đã tìm thấy robot với tên: {name}")
                        break
                else:
                    rospy.logwarn_throttle(5.0, f"Không tìm thấy robot '{self.robot_name}' trong Gazebo models. Models hiện tại: {msg.name}")
                    return
            
            # Lấy vị trí và hướng của robot
            pose = msg.pose[idx]
            twist = msg.twist[idx]
            
            # Lấy timestamp hiện tại
            current_time = rospy.Time.now()
            
            # Chuẩn bị thông điệp PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time
            pose_stamped.header.frame_id = self.output_frame
            pose_stamped.pose = pose
            
            # Xuất ra topic pose
            self.pose_pub.publish(pose_stamped)
            
            # Thêm vào quỹ đạo và xuất ra topic path
            self.path.header.stamp = current_time
            self.path.poses.append(pose_stamped)
            self.path_pub.publish(self.path)
            
            # Chuẩn bị thông điệp Odometry
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = self.output_frame
            odom.child_frame_id = "base_footprint"
            odom.pose.pose = pose
            odom.twist.twist = twist
            
            # Xuất ra topic odom
            self.odom_pub.publish(odom)
            
            # Xuất TF transform
            self.tf_broadcaster.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                current_time,
                "base_footprint",
                "ground_truth"
            )
            
            # Xuất thêm một TF từ map đến ground truth frame
            self.tf_broadcaster.sendTransform(
                (0, 0, 0),
                (0, 0, 0, 1),
                current_time,
                "ground_truth",
                self.output_frame
            )
            
        except Exception as e:
            rospy.logerr(f"Lỗi khi xử lý dữ liệu từ Gazebo: {e}")
    
    def run(self):
        """
        Chạy node
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GazeboGroundTruth()
        node.run()
    except rospy.ROSInterruptException:
        pass