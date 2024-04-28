#!/usr/bin/env python

# Imports
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

# Classes
class WaypointTFPublisher:
    """
    Class definition for a tf waypoint publisher.
    """
    def __init__(self, velocity):
        """
        Class constructor.
        """
        # Set initial values
        self.velocity = velocity
        
        # Create odometry publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # Create odometry message
        self.odom = Odometry()
        # Set header of message
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = 'map'  # Assuming your initial pose is in the map frame
        
        # Initialise broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        # Initialise waypoint list
        self.waypoints = []

    def calculate_yaw(self, p1, p2):
        """
        Compute yaw angle for the waypoint, given the inner and outer edge.
        """
        # Compute differences in x an y
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        # Use trigonometry to return yaw angle
        return math.atan2(dy, dx)

    def calculate_distance(self, p1, p2):
        """
        Compute Euclidean distnace.
        """
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def path_callback(self, path_msg):
        """
        Callback function for path subscriber.
        """
        print("Path message received")
        # Extract poses from path msg
        self.waypoints = path_msg.poses
        # If non-empty, then publish waypoints
        if self.waypoints:
            self.publish_tf_waypoints()

    def publish_tf_waypoints(self):
        """
        Publishes waypoints as odometry messages.
        """
        while not rospy.is_shutdown() and len(self.waypoints) > 1:
            # Iterate through waypoint list
            for i in range(len(self.waypoints) - 1):
                # Extract current and next waypoint
                p1 = self.waypoints[i]
                p2 = self.waypoints[i + 1]
                
                # Compute yaw/ heading angle
                yaw = self.calculate_yaw(p1, p2)
                # Compute distance between waypoints
                distance = self.calculate_distance(p1, p2)
                if distance == 0:
                    continue
                # Define a fixed velocity
                rate = rospy.Rate(self.velocity / distance)

                # Broadcast transform containing pose (position + orientation)
                self.tf_broadcaster.sendTransform(
                    (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z+0.36),
                    (p1.pose.orientation.x, p1.pose.orientation.y, p1.pose.orientation.z, p1.pose.orientation.w),
                    rospy.Time.now(),
                    "base_link",
                    "map"
                )
                
                # Set odometry message values
                self.odom.pose.pose.position.x = p1.pose.position.x
                self.odom.pose.pose.position.y = p1.pose.position.y
                self.odom.pose.pose.position.z = 0.36
                self.odom.pose.pose.orientation.x = p1.pose.orientation.x
                self.odom.pose.pose.orientation.y = p1.pose.orientation.y
                self.odom.pose.pose.orientation.z = p1.pose.orientation.z
                self.odom.pose.pose.orientation.w = p1.pose.orientation.w
                # Publish current robot pose to /odom
                self.odom_pub.publish(self.odom)

                rate.sleep()

def main():
    """
    Main function.
    """
    # Initialise node
    rospy.init_node('tf_publisher')
    velocity = 30

    # Instantiate waypoint publisher object
    tf_publisher = WaypointTFPublisher(velocity)
    # Create subscriber to path
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, tf_publisher.path_callback)
    print("Waiting for Path message")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
