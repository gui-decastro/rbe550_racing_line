#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


def odom_callback(odom_msg):
    global tf_broadcaster

    # Create a TF transform from /odom to /base_link
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "map"
    tf.child_frame_id = "base_footprint"
    tf.transform.translation.x = odom_msg.pose.pose.position.x
    tf.transform.translation.y = odom_msg.pose.pose.position.y
    tf.transform.translation.z = odom_msg.pose.pose.position.z
    tf.transform.rotation = odom_msg.pose.pose.orientation

    # Publish the TF transform
    tf_broadcaster.sendTransform(tf)

def main():
    # Initialise node
    rospy.init_node('odom_to_tf')
    
    global tf_broadcaster  # Declare tf_broadcaster as global
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Gazebo publishes to /odom, now convert that odometry to tf so the state can be reflected in RViz as well
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass