#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
import math

class WaypointTFPublisher:
    def __init__(self, velocity):
        self.velocity = velocity
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.waypoints = []

    def calculate_yaw(self, p1, p2):
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        return math.atan2(dy, dx)

    def calculate_distance(self, p1, p2):
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def path_callback(self, path_msg):
        print("Path message received")
        self.waypoints = path_msg.poses
        if self.waypoints:
            self.publish_tf_waypoints()

    def publish_tf_waypoints(self):
        

        while not rospy.is_shutdown() and len(self.waypoints) > 1:
            for i in range(len(self.waypoints) - 1):
                p1 = self.waypoints[i]
                p2 = self.waypoints[i + 1]

                yaw = self.calculate_yaw(p1, p2)

                distance = self.calculate_distance(p1, p2)
                if distance == 0:
                    continue
                rate = rospy.Rate(self.velocity / distance)

                self.tf_broadcaster.sendTransform(
                    (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z),
                    (p1.pose.orientation.x, p1.pose.orientation.y, p1.pose.orientation.z, p1.pose.orientation.w),
                    rospy.Time.now(),
                    "base_link",
                    "map"
                )
                rate.sleep()

def main():
    rospy.init_node('tf_publisher')
    velocity = 7

    tf_publisher = WaypointTFPublisher(velocity)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, tf_publisher.path_callback)
    print("Waiting for Path message")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
