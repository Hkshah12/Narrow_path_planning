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

    # Copy the child frame ID from the odometry message
    pose_msg.header.frame_id = nav_msg.child_frame_id
    # pose_msg.header.child_frame_id = nav_msg.child_frame_id  # The child frame

    # Use the covariance matrix from the Odometry message
    pose_msg.pose.covariance = nav_msg.pose.covariance

    # Publish the PoseWithCovarianceStamped message
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('odom_to_pose_converter', anonymous=True)

    # Publisher for the PoseWithCovarianceStamped message
    pose_pub = rospy.Publisher("/pose_with_covariance", PoseWithCovarianceStamped, queue_size=10)

    # Subscribe to the RTAB-Map odometry topic
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)

    # Keep the node running
    rospy.spin()
