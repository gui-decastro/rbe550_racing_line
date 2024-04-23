#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import math
import time

class WaypointTFPublisher:
    def __init__(self, waypoints, velocity, spacing):
        self.waypoints = waypoints
        self.velocity = velocity
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.spacing = spacing

    def calculate_yaw(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)
    
    def publish_tf_waypoints(self):
        rospy.init_node('tf_publisher')
        rate = rospy.Rate(self.velocity/self.spacing)

        for i, (x, y) in enumerate(self.waypoints):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            yaw = self.calculate_yaw(p1, p2)

            self.tf_broadcaster.sendTransform(
                (x, y, 0.0),
                tf.transformations.quaternion_from_euler(0, 0, yaw),
                rospy.Time.now(),
                "base_link",
                "world"
            )
            print(rospy.Time.now())
            rate.sleep()

def generate_circle_waypoints(radius, spacing):
    waypoints = []
    angle = 0.0
    while angle < 2*math.pi:
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        waypoints.append((x, y))
        angle += spacing / radius  
    return waypoints

def main():

    radius = 25.0 
    spacing = 1.0  
    velocity = 2  

    waypoints = generate_circle_waypoints(radius, spacing)

    tf_publisher = WaypointTFPublisher(waypoints, velocity, spacing)
    tf_publisher.publish_tf_waypoints()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
