#!/usr/bin/env python3

# https://github.com/ahmedmoawad124/Self-Driving-Vehicle-Control/blob/master/README.md

import pickle
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from teb_local_planner.msg import FeedbackMsg

def local_planner_feedback_callback(msg):
    print("Local planner feedback received...")

    global throttle_pub
    global brake_pub
    global steering_pub
    
    # Implement your control logic here to convert teb_feedback to control commands
    throttle_cmd = Float64()
    brake_cmd = Float64()
    steering_cmd = Float64()

    # Example: Set throttle_cmd, brake_cmd, and steering_cmd based on teb_feedback
    throttle_cmd.data = 0.5  # Example throttle value
    brake_cmd.data = 0.0     # Example brake value
    steering_cmd.data = 0.1  # Example steering angle

    # Publish control commands
    throttle_pub.publish(throttle_cmd)
    brake_pub.publish(brake_cmd)
    steering_pub.publish(steering_cmd)

def controller_node():
    rospy.init_node('controller_node')

    # Subscribe to teb_feedback topic
    rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_feedback', FeedbackMsg, local_planner_feedback_callback)


    # Publishers for control commands
    global throttle_pub
    global brake_pub
    global steering_pub
    throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
    brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass


# rospy.init_node('controller')

# # Define goal publisher
# r = rospy.Rate(10) # 10hz

# # Publishers (output of controller)
# throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
# steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)

# throttle_cmd = Float64()
# throttle_cmd.data = 0.1

# steering_cmd = Float64()
# steering_cmd.data = 0.7

# while not rospy.is_shutdown():
#     r.sleep()
#     throttle_pub.publish(throttle_cmd)
#     steering_pub.publish(steering_cmd)

# rospy.sleep(1)
