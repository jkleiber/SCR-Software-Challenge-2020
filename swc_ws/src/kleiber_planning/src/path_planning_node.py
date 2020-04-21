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
GRID_SIZE = 0.1     # Map block size in meters
LIDAR_RANGE = 10
MAP_WIDTH = 2*int(LIDAR_RANGE / GRID_SIZE)
MAP_HEIGHT = 2*int(LIDAR_RANGE / GRID_SIZE)
map_center = int(MAP_HEIGHT / 2)

# Cost map
cost_map = None

# Global waypoint target (D* goal position)
active_wpt = None

# Local target to increase speed
local_goal = None

# Origin waypoint and pose offset
start_lat = None
start_lon = None
start_x = None
start_y = None

def cost(pos):
    if cost_map is None or local_goal is None:
        return 100
    else:
        return cost_map[pos[0] * MAP_WIDTH + pos[1]]

def robot_pose_callback(data):
    global robot_pose, local_goal
    robot_pose = data

    # Try to find a goal that is close by so path planning is faster
    if active_wpt is not None:
        local_goal = None
        i = 0
        dist = 3 / GRID_SIZE
        while cost(local_goal) > 0 and dist <= (10 / GRID_SIZE):
            dist = (3+i*0.5) / GRID_SIZE    # Lookahead distance

            # Find robot position in cells
            robot_x = int(robot_pose.pose.position.x / GRID_SIZE)
            robot_y = int(robot_pose.pose.position.y / GRID_SIZE)

            # Find heading from robot to goal
            dx = active_wpt[0] - robot_x
            dy = active_wpt[1] - robot_y

            # If the distance is greater than the distance to goal, set the waypoint to a fixed target
            if dist**2 > (dx**2 + dy**2):
                local_goal = (map_center - dx, map_center - dy)
                return

            hdg = math.atan2(dy, dx)

            x_goal = int(round(map_center - dist * math.cos(hdg)))
            y_goal = int(round(map_center - dist * math.sin(hdg)))
            local_goal = (x_goal, y_goal)
            # print((map_center, map_center), local_goal, hdg)

            # Increase lookahead distance if needed
            i += 1

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
    x_bin = int(round(x / GRID_SIZE))
    y_bin = int(round(y / GRID_SIZE))

    # Set active waypoint
    active_wpt = (x_bin, y_bin)


def path_point_to_global_pose_stamped(robot_pos, pp0, pp1):
    # Convert path grid cell into x,y
    pp0 = map_center - pp0
    pp1 = map_center - pp1
    x = (robot_pos[0] + pp0) * GRID_SIZE
    y = (robot_pos[1] + pp1) * GRID_SIZE

    # Make a PoseStamped for the path point
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = x
    point.y = y
    pose_stamped.pose.position = point

    return pose_stamped

def path_plan(c_space):
    global planner, map_init, path_failed, cost_map

    # We can't plan without a map, a start, and a goal
    if cost_map is None or robot_pose is None or active_wpt is None or local_goal is None:
        return

    # Robot pose in local frame is fixed
    robot_pos = (map_center, map_center)

    # Global robot pose -> global grid cells for a global map
    global_robot_pos = (int(robot_pose.pose.position.x / GRID_SIZE), int(robot_pose.pose.position.y / GRID_SIZE))

    # Get the waypoint coordinates
    wpt_x = active_wpt[0]
    wpt_y = active_wpt[1]

    # Find distance from robot to goal in cells
    dx = wpt_x - global_robot_pos[0]
    dy = wpt_y - global_robot_pos[1]

    # If we are really close to the waypoint, there are no obstacles so publish only the waypoint position
    # Short-circuit the path planner in this case
    # if (dx**2 + dy**2) <= (3**2 / GRID_SIZE):
    #     print("SHORTCUT: Path Planner not needed")
    #     path_point = (map_center - dx, map_center - dy)
    #     global_path = Path()

    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "base_link"

    #     global_path.header = header
    #     global_path.poses = [path_point_to_global_pose_stamped(global_robot_pos, path_point[0], path_point[1])]

    #     path_pub.publish(global_path)
    #     map_init = False
    #     return

    # convert cost_map to list
    active_cost_map = list(cost_map)

    # The cost to the goal is always 0
    # Inflate the non obstacles around the active waypoint
    # inflation = int(2 / GRID_SIZE)
    # for i in range(wpt_x - inflation, wpt_x + inflation):
    #     for j in range(wpt_y - inflation, wpt_y + inflation):
    #         active_cost_map[i * MAP_WIDTH + j] = 0

    # Robot position should always have no obstacles on it
    active_cost_map[robot_pos[0] * MAP_WIDTH + robot_pos[1]] = 0

    # Reset the path
    path = None

    # MOVING TARGET D*LITE
    # if map_init is False:
    if True:
        planner.initialize(MAP_WIDTH, MAP_HEIGHT, robot_pos, local_goal, active_cost_map)
        path = planner.plan()
        map_init = True
    else:
        path = planner.replan(robot_pos, local_goal, active_cost_map)

    # Publish the path if it exists
    if path is not None:
        print("SUCCESS: Path Found")
        global_path = Path()

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        global_path.header = header
        global_path.poses = [path_point_to_global_pose_stamped(global_robot_pos, path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse backwards path

        path_pub.publish(global_path)
    else:
        print("FAIL: Path Not Found")
        map_init = False


def mt_dstar_node():
    global start_lat, start_lon, start_x, start_y

    # Setup node
    rospy.init_node("path_planning_node")

    start_lat = rospy.get_param("start_lat")
    start_lon = rospy.get_param("start_lon")
    start_x = rospy.get_param("start_x")
    start_y = rospy.get_param("start_y")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/local_map", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    pose_sub = rospy.Subscriber("/robot/pose", PoseStamped, robot_pose_callback, queue_size=1)
    wpt_sub = rospy.Subscriber("/task/goal", Gps, waypoint_callback, queue_size=1)

    # Make a timer to publish new paths
    timer = rospy.Timer(rospy.Duration(secs=0.25), path_plan, oneshot=False)

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
