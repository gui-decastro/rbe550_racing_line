#!/usr/bin/env python3

# https://github.com/ahmedmoawad124/Self-Driving-Vehicle-Control/blob/master/README.md

import pickle
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from teb_local_planner.msg import FeedbackMsg
from teb_local_planner.msg import TrajectoryMsg


lin_vel = 0.0
ang_vel = 0.0
err_cum = 0.0
err_prev = 0.0


def controller(target_vel):

    global err_cum
    global err_prev

    # Set error for current timesteps
    err_current = target_vel - lin_vel.x
    # Set timestep
    dt = 1

    # Set controller gains
    Kp = 0.5
    Ki = 0.1
    Kd = 0.2

    # Cumulative sum of errors (for integral control)
    err_cum = err_cum + err_current*dt

    # Proportional control equation
    up = Kp * err_current
    # Integral control equation
    ui = Ki * err_cum
    # Derivative control equation
    ud = Kd * (err_current - err_prev)/dt

    # PID Control equation
    u = up + ui + ud

    # Save current timestep error (for derivative control)
    err_prev = err_current

    return u

def local_planner_feedback_callback(msg):
    print("Local planner feedback received...")

    idx = msg.selected_trajectory_idx
    print("Trajectory idx: ",idx)

    print("x:",msg.trajectories[idx].trajectory[0].velocity.linear.x,"y:",msg.trajectories[idx].trajectory[0].velocity.linear.y,"z:",msg.trajectories[idx].trajectory[0].velocity.linear.z)

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

    # Call PID controller
    throttle_val = controller(msg.trajectories[idx].trajectory[0].velocity.linear.x)

    print("throttle_val: ",throttle_val)
    throttle_cmd.data = throttle_val

    # Publish control commands
    throttle_pub.publish(throttle_cmd)
    brake_pub.publish(brake_cmd)
    steering_pub.publish(steering_cmd)


def odometry_callback(msg):

    global lin_vel 
    global  ang_vel

    lin_vel = msg.twist.twist.linear
    ang_vel = msg.twist.twist.angular
    return


def controller_node():
    rospy.init_node('controller_node')

    # Subscribe to teb_feedback topic
    rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_feedback', FeedbackMsg, local_planner_feedback_callback)
    # Subscribe to audibot odom
    rospy.Subscriber('/odom', Odometry, odometry_callback)


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
