#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Transform, Point, Quaternion
from std_msgs.msg import Header
import threading

class TrajectoryPublisher:
    """
    Xuất quỹ đạo từ thuật toán SLAM và ground truth ra các ROS topics
    """
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=True)
        
        # Đọc các tham số ROS
        self.gt_frame = rospy.get_param('~gt_frame', 'map')
        self.slam_frame = rospy.get_param('~slam_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        self.out_topic = rospy.get_param('~out_topic', '/ground_truth/path')
        self.slam_topic = rospy.get_param('~slam_topic', '/slam/trajectory')
        self.publish_rate = rospy.get_param('~rate', 10.0)  # Hz
        
        self.rate = rospy.Rate(self.publish_rate)
        
        # Tạo publishers cho các quỹ đạo
        self.path_pub = rospy.Publisher(self.out_topic, Path, queue_size=1)
        self.slam_path_pub = rospy.Publisher(self.slam_topic, Path, queue_size=1)
        
        # Lưu trữ quỹ đạo
        self.path = Path()
        self.path.header.frame_id = self.gt_frame
        
        self.slam_path = Path()
        self.slam_path.header.frame_id = self.slam_frame
        
        # Thiết lập TF listener
        self.tf_listener = tf.TransformListener()
        
        # Theo dõi map để biết khi có update
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.latest_map_time = rospy.Time(0)
        
        rospy.loginfo("Trajectory publisher initialized with params:")
        rospy.loginfo(f" - Ground truth frame: {self.gt_frame}")
        rospy.loginfo(f" - SLAM frame: {self.slam_frame}")
        rospy.loginfo(f" - Robot frame: {self.robot_frame}")
        rospy.loginfo(f" - Output topic: {self.out_topic}")
        rospy.loginfo(f" - SLAM trajectory topic: {self.slam_topic}")
        rospy.loginfo(f" - Publishing rate: {self.publish_rate} Hz")
    
    def map_callback(self, map_msg):
        """
        Callback khi có bản đồ mới
        """
        self.latest_map_time = map_msg.header.stamp
    
    def update_ground_truth_path(self):
        """
        Cập nhật quỹ đạo ground truth
        """
        try:
            current_time = rospy.Time.now()
            
            # Đợi và lấy transform
            if self.tf_listener.waitForTransform(self.gt_frame, self.robot_frame, current_time, rospy.Duration(0.1)):
                (trans, rot) = self.tf_listener.lookupTransform(self.gt_frame, self.robot_frame, current_time)
                
                # Tạo pose message và thêm vào path
                pose = PoseStamped()
                pose.header.stamp = current_time
                pose.header.frame_id = self.gt_frame
                pose.pose.position = Point(trans[0], trans[1], trans[2])
                pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
                
                self.path.poses.append(pose)
                self.path.header.stamp = current_time
                self.path_pub.publish(self.path)
                return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get ground truth transform: {e}")
        return False
    
    def update_slam_path(self):
        """
        Cập nhật quỹ đạo từ thuật toán SLAM
        """
        try:
            current_time = rospy.Time.now()
            
            # Đợi và lấy transform
            if self.tf_listener.waitForTransform(self.slam_frame, self.robot_frame, current_time, rospy.Duration(0.1)):
                (trans, rot) = self.tf_listener.lookupTransform(self.slam_frame, self.robot_frame, current_time)
                
                # Tạo pose message và thêm vào path
                pose = PoseStamped()
                pose.header.stamp = current_time
                pose.header.frame_id = self.slam_frame
                pose.pose.position = Point(trans[0], trans[1], trans[2])
                pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
                
                self.slam_path.poses.append(pose)
                self.slam_path.header.stamp = current_time
                self.slam_path_pub.publish(self.slam_path)
                return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get SLAM transform: {e}")
        return False
    
    def run(self):
        """
        Vòng lặp chính để liên tục cập nhật và xuất quỹ đạo
        """
        rospy.loginfo("Trajectory publisher starting...")
        
        try:
            # Đợi một khoảng thời gian để các hệ thống SLAM khởi động
            rospy.sleep(2.0)
            
            while not rospy.is_shutdown():
                # Cập nhật và xuất các quỹ đạo
                gt_updated = self.update_ground_truth_path()
                slam_updated = self.update_slam_path()
                
                # Log trạng thái cập nhật (mỗi 10 giây)
                if rospy.Time.now().to_sec() % 10 < 0.1:
                    msg = []
                    if gt_updated:
                        msg.append(f"Ground truth: {len(self.path.poses)} poses")
                    if slam_updated:
                        msg.append(f"SLAM: {len(self.slam_path.poses)} poses")
                    if msg:
                        rospy.loginfo(", ".join(msg))
                
                self.rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("Trajectory publisher shutting down")

if __name__ == "__main__":
    try:
        node = TrajectoryPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass