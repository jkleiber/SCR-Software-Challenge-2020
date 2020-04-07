#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
import copy
import numpy as np
from field_dstar import mt_dstar_lite


SHOW_PLOTS = False

path_pub = rospy.Publisher("/path", Path, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()


# Localization tracking
prev_state = (0, 0)  # x, y
GRID_SIZE = 0.05      # Map block size in meters

# Path tracking
path_seq = 0

# Cost map
cost_map = None

# Latest pose update
robot_pose = None

# Global waypoint target (D* goal position)
active_wpt = None

def robot_pose_callback(data):
    global robot_pose
    robot_pose = data

def c_space_callback(c_space):
    global cost_map
    cost_map = c_space.data

def waypoint_callback(waypoint):
    global active_wpt

    # Project GPS onto grid
    x = waypoint.latitude
    y = waypoint.longitude

    # Set active waypoint
    active_wpt = (x, y)


def path_point_to_global_pose_stamped(robot_pos, pp0, pp1):
    # global path
    x = pp0 * GRID_SIZE
    y = pp1 * GRID_SIZE

    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = x
    point.y = y
    pose_stamped.pose.position = point

    return pose_stamped

def path_plan(c_space):
    global planner, map_init, path_failed, prev_state, path_seq

    if cost_map is None or robot_pose is None or active_wpt is None:
        return

    # Reset the path
    path = None

    robot_pos = (robot_pose.pose.position.x / GRID_SIZE, robot_pose.pose.position.y / GRID_SIZE)

    # MOVING TARGET D*LITE
    planner.initialize(200, 200, robot_pos, active_wpt, cost_map)
    path = planner.plan()
    map_init = True

    if path is not None:
        global_path = Path()

        header = Header()
        header.seq = path_seq
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        path_seq += 1

        global_path.header = header
        global_path.poses = [path_point_to_global_pose_stamped(robot_pos, path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse path becuz its backwards lol

        path_pub.publish(global_path)


def mt_dstar_node():
    # Setup node
    rospy.init_node("path_planning_node")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/map", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    pose_sub = rospy.Subscriber("/robot/pose", PoseStamped, robot_pose_callback)

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
