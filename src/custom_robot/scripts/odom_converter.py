#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from math import sin, cos

class OdomConverter:
    def __init__(self):
        rospy.init_node('odom_converter')
        
        # Parameters
        self.publish_tf = True
        
        # Publishers
        self.odom_pub = rospy.Publisher('odom_converted', Odometry, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('odom', Point, self.odom_callback)
        
        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("Odometry converter node started")
    
    def odom_callback(self, point_msg):
        # Extract position and orientation from the Point message
        x = point_msg.x
        y = point_msg.y
        theta = point_msg.z  # Using z to store theta (yaw)
        
        # Create quaternion for orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        
        # Current time
        current_time = rospy.Time.now()
        
        # Create and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Publish transform if enabled
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                (x, y, 0),
                quat,
                current_time,
                "base_link",
                "odom"
            )

if __name__ == '__main__':
    try:
        OdomConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass