#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# Publisher for the converted TransformStamped messages
pub = None

def tf_callback(tf_msg):
    # Loop over each TransformStamped message in the TFMessage
    for transform in tf_msg.transforms:
        # Create a new TransformStamped message
        transform_stamped = TransformStamped()
        
        # Copy data from TFMessage to TransformStamped
        transform_stamped.header = transform.header
        transform_stamped.child_frame_id = transform.child_frame_id
        transform_stamped.transform = transform.transform

        # Publish the converted TransformStamped message
        pub.publish(transform_stamped)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('tf_extractor')

    # Publisher to send TransformStamped messages on the new topic
    pub = rospy.Publisher('/tf_converted', TransformStamped, queue_size=10)

    # Subscriber to listen to the original /tf topic with TFMessage
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    # Spin to keep the node active and processing callbacks
    rospy.spin()

