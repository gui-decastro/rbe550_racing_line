#!/usr/bin/env python

# Based off ROS wiki tutorial:
# http://wiki.ros.org/teb_local_planner/Tutorials/Inspect%20optimization%20feedback

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter

def feedback_callback(data):
  global trajectory

  if not data.trajectories: # empty
    trajectory = []
    return
  trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
  
  
def plot_velocity_profile(fig, ax_v, ax_omega, ax_accel, t, v, omega, a):
  # plot translational velocity
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  # plot rotation velocity
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  # plot translational acceleration
  ax_accel.cla()
  ax_accel.grid()
  ax_accel.set_ylabel('Acceleration [m/s^2]')
  ax_accel.set_xlabel('Time [s]')
  ax_accel.plot(t, a, '-bx')

  fig.canvas.draw()

  
  
def velocity_plotter():
  global trajectory
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  
  topic_name = "/move_base/TebLocalPlannerROS/teb_feedback" # define feedback topic here!
  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) 

  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # two subplots sharing the same t axis
#   fig, (ax_v, ax_omega) = plotter.subplots(2, sharex=True)
  fig, (ax_v, ax_omega, ax_accel) = plotter.subplots(3, sharex=True)
  fig.set_size_inches(8, 6)
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  
  # Initialize values for acceleration calculation
  v_prev = 0.0
  t_prev = 0.0
  
  while not rospy.is_shutdown():
    
    t = []
    v = []
    omega = []
    a = []
    
    for point in trajectory:
      t_curr = point.time_from_start.to_sec()
      v_curr = point.velocity.linear.x
    #   print(f"v_curr: {v_curr}")
      
      # TODO: fix later because this is a bad patch
      if t_curr > 0.0 and v_curr == 0.0:
        break
      
      t.append(t_curr)
      v.append(v_curr)
      omega.append(point.velocity.angular.z)
      
      # avoid dividing by 0 on first iteration
      if t_curr == 0.0:
        accel = 0
      else:
        accel = (v_curr - v_prev) / (t_curr - t_prev)

      a.append(accel)

      # update values for next iteration
      t_prev = t_curr
      v_prev = v_curr
          
    plot_velocity_profile(fig, ax_v, ax_omega, ax_accel, np.asarray(t), np.asarray(v), np.asarray(omega), np.asarray(a))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass