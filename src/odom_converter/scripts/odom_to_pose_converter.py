#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_callback(nav_msg):
    # Create a new PoseWithCovarianceStamped message
    pose_msg = PoseWithCovarianceStamped()

    # Copy the header from the odometry message
    pose_msg.header = nav_msg.header

    # Set the pose part of PoseWithCovarianceStamped from the Odometry message
    pose_msg.pose.pose = nav_msg.pose.pose

    # Define a static covariance matrix
    pose_msg.pose.covariance = [
        0.05, 0, 0, 0, 0, 0,
        0, 0.05, 0, 0, 0, 0,
        0, 0, 0.05, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    ]

    # Publish the PoseWithCovarianceStamped message
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('odom_to_pose_converter', anonymous=True)

    # Subscribe to the RTAB-Map odometry topic
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)

    # Publisher for the PoseWithCovarianceStamped message
    pose_pub = rospy.Publisher("/pose_with_covariance", PoseWithCovarianceStamped, queue_size=10)

    # Keep the node running
    rospy.spin()
