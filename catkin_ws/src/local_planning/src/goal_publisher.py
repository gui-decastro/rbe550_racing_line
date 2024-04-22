import pickle
import rospy
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
rospy.init_node('goal_pub')
r = rospy.Rate(10) # 10hz

file_path = "waypoints.pickle"

with open(file_path , 'rb') as f:
    waypoints = pickle.load(f)

print(waypoints)
# while not rospy.is_shutdown():

for i in range(len(waypoints)):

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = -(waypoints[i][0][0]+waypoints[i][1][0])/200
    goal.pose.position.y = (waypoints[i][0][1]+waypoints[i][1][1])/200
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.sleep(1)
    pub.publish(goal)

print("List of waypoints complete")
    
# rospy.sleep(5)
r.sleep()