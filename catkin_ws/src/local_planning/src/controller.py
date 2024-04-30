#!/usr/bin/env python3

import pickle
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

rospy.init_node('controller')

# Define goal publisher
r = rospy.Rate(10) # 10hz

# Publishers (output of controller)
throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)

throttle_cmd = Float64()
throttle_cmd.data = 0.1

steering_cmd = Float64()
steering_cmd.data = 0.7

while not rospy.is_shutdown():
    r.sleep()
    throttle_pub.publish(throttle_cmd)
    steering_pub.publish(steering_cmd)

rospy.sleep(1)
