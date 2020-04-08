#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
import copy
import numpy as np

from swc_msgs.msg import Gps
from field_dstar import mt_dstar_lite


SHOW_PLOTS = False

path_pub = rospy.Publisher("/path", Path, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()

# Localization tracking
robot_pose = None    # Latest PoseStamped update
GRID_SIZE = 0.25     # Map block size in meters

# Cost map
cost_map = None

# Global waypoint target (D* goal position)
active_wpt = None

# Origin waypoint and pose offset
start_lat = None
start_lon = None
start_x = None
start_y = None

def robot_pose_callback(data):
    global robot_pose
    robot_pose = data

def c_space_callback(c_space):
    global cost_map
    cost_map = c_space.data

def waypoint_callback(waypoint):
    """ Project active GPS goal onto 2D grid """
    global active_wpt

    # Get lat/lon diffs
    dLat = waypoint.latitude - start_lat
    dLon = waypoint.longitude - start_lon

    # Find x and y coordinates
    # HACK: Assume that our coordinate frame is perfectly aligned with north (i.e. x is due north)
    # HACK: Assume that a degree of difference in lat/lon is 111.1111 km
    x = start_x + (dLat * 110944.33)
    y = start_y + (dLon * 91058.93)

    # Bin the x and y coordinate into a grid cell
    x_bin = int(x / GRID_SIZE)
    y_bin = int(y / GRID_SIZE)

    # Set active waypoint
    active_wpt = (x_bin, y_bin)


def path_point_to_global_pose_stamped(robot_pos, pp0, pp1):
    # Convert path grid cell into x,y
    x = pp0 * GRID_SIZE
    y = pp1 * GRID_SIZE

    # Make a PoseStamped for the path point
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = x
    point.y = y
    pose_stamped.pose.position = point

    return pose_stamped

def path_plan(c_space):
    global planner, map_init, path_failed

    # We can't plan without a map, a start, and a goal
    if cost_map is None or robot_pose is None or active_wpt is None:
        return

    # Reset the path
    path = None

    # Convert the robot's (x,y) pose into grid cells
    robot_pos = (int(robot_pose.pose.position.x / GRID_SIZE), int(robot_pose.pose.position.y / GRID_SIZE))
    print(active_wpt)

    # MOVING TARGET D*LITE
    planner.initialize(100, 200, robot_pos, active_wpt, cost_map)
    path = planner.plan()
    map_init = True

    print("path planning complete")

    # Publish the path if it exists
    if path is not None:
        print("Path Found")
        global_path = Path()

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        global_path.header = header
        global_path.poses = [path_point_to_global_pose_stamped(robot_pos, path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse backwards path

        path_pub.publish(global_path)


def mt_dstar_node():
    global start_lat, start_lon, start_x, start_y

    # Setup node
    rospy.init_node("path_planning_node")

    start_lat = rospy.get_param("start_lat")
    start_lon = rospy.get_param("start_lon")
    start_x = rospy.get_param("start_x")
    start_y = rospy.get_param("start_y")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/map", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    pose_sub = rospy.Subscriber("/robot/pose", PoseStamped, robot_pose_callback, queue_size=1)
    wpt_sub = rospy.Subscriber("/task/goal", Gps, waypoint_callback, queue_size=1)

    # Make a timer to publish new paths
    timer = rospy.Timer(rospy.Duration(secs=0.5), path_plan, oneshot=False)

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
