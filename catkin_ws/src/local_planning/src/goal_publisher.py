import pickle
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped


# Define parameters to convert image coordinates to /map coordinates
racetrack_image_width = 940
racetrack_image_height = 776
resolution = 0.25 # all these values are hard coded in. Need to be changed if input map is updated

rospy.init_node('start_and_goal_pose_publisher')

# Define Initial node publisher (publishes initial state to be used by planner)
start_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
start = PoseWithCovarianceStamped()
start.header.stamp = rospy.Time.now()
start.header.frame_id = 'map'  # Assuming your initial pose is in the map frame

# Define goal publisher
goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
r = rospy.Rate(10) # 10hz
goal = PoseStamped()

goal.header.seq = 1
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"


# Import waypoint data
file_path = "waypoints3.pickle"
with open(file_path , 'rb') as f:
    waypoints = pickle.load(f)

# manually found orientation at every state. This is for waypoints in waypoints3.pickle! Will not work with others
orientation_data = [
    {'x': 0.0, 'y': 0.0, 'z': -0.9997214612303805, 'w': 0.023600846497379174},
    {'x': 0.0, 'y': 0.0, 'z': 0.6604921555499723, 'w': 0.7508329457721945},
    {'x': 0.0, 'y': 0.0, 'z': 0.362523128906254, 'w': 0.9319747748775283},
    {'x': 0.0, 'y': 0.0, 'z': -0.16217813996814665, 'w': 0.9867614964703842},
    {'x': 0.0, 'y': 0.0, 'z': -0.19084445240159337, 'w': 0.9816202906356082},
    {'x': 0.0, 'y': 0.0, 'z': 0.9998699982157817, 'w': 0.016124102082680537},
    {'x': 0.0, 'y': 0.0, 'z': -0.923879168696372, 'w': 0.3826843106908102},
    {'x': 0.0, 'y': 0.0, 'z': -0.6470126062719062, 'w': 0.76247930288319},
    {'x': 0.0, 'y': 0.0, 'z': -0.030989933369501798, 'w': 0.9995196966692321},
    {'x': 0.0, 'y': 0.0, 'z': -0.6004957906413554, 'w': 0.799627916860094},
    {'x': 0.0, 'y': 0.0, 'z': -0.9999999999999062, 'w': 4.33125768201249e-07},
    {'x': 0.0, 'y': 0.0, 'z': 0.9999049248336904, 'w': 0.013789173047426002}
]

# print(waypoints)
# while not rospy.is_shutdown():
rospy.sleep(5) # add this or else first iteration of for loop doenst publish anything

# every iteration sets waypoints[i] as initial pose, and waypoints[i+1] as goal pose
for i in range(len(waypoints)-1):
    print(i)
    x_start_image_coords = (waypoints[i][0][0]+waypoints[i][1][0])/2
    y_start_image_coords = (waypoints[i][0][1]+waypoints[i][1][1])/2

    start.pose.pose.position.x = x_start_image_coords * resolution
    start.pose.pose.position.y = (racetrack_image_height - y_start_image_coords) * resolution
    start.pose.pose.position.z = 0.0

    start_orientation = orientation_data[i]
    print(f"start {start_orientation}")
    start.pose.pose.orientation.x = start_orientation['x']
    start.pose.pose.orientation.y = start_orientation['y']
    start.pose.pose.orientation.z = start_orientation['z']
    start.pose.pose.orientation.w = start_orientation['w']
    start_pub.publish(start)


    # The waypoints come as coordinates from an opencv image. A conversion needs to be done to convert it to map coordinates
    x_goal_image_coords = (waypoints[i+1][0][0]+waypoints[i+1][1][0])/2
    y_goal_image_coords = (waypoints[i+1][0][1]+waypoints[i+1][1][1])/2

    goal.pose.position.x = x_goal_image_coords * resolution
    goal.pose.position.y = (racetrack_image_height - y_goal_image_coords) * resolution
    goal.pose.position.z = 0.0

    goal_orientation = orientation_data[i+1]
    print(f"goal {goal_orientation}")
    goal.pose.orientation.x = goal_orientation['x']
    goal.pose.orientation.y = goal_orientation['y']
    goal.pose.orientation.z = goal_orientation['z']
    goal.pose.orientation.w = goal_orientation['w']

    goal_pub.publish(goal)

    print(f"({start.pose.pose.position.x},{start.pose.pose.position.y}) -> ({goal.pose.position.x},{goal.pose.position.x})")
    rospy.sleep(5)

print("List of waypoints complete")
    
# rospy.sleep(5)
r.sleep()