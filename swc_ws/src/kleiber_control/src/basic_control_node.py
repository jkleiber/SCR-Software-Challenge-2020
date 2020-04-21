#!/usr/bin/env python2

import rospy

from math import atan2, degrees, radians, fmod

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from swc_msgs.msg import Control, RobotState, Gps
from gps_calc import measure_gps

# Path data
desired_path = None

# State data
robot_state = None

# Control data
ctrl = Control()
ctrl_pub = rospy.Publisher("/kleiber/control", Control, queue_size=1)

# Goal data
goal = Gps()
dist_to_goal = 100

# Grid size constant
GRID_SIZE = 0.1


def state_estimate_callback(state):
    global robot_state, dist_to_goal
    robot_state = state

    # Get the distance to goal
    if goal is not None:
        dist_to_goal = measure_gps(robot_state.latitude, robot_state.longitude, goal.latitude, goal.longitude)

def path_callback(path):
    global desired_path
    desired_path = path

def goal_callback(gps):
    global goal
    goal = gps


def angle_diff(angle1, angle2):
    a = degrees(angle1)
    b = degrees(angle2)

    dif = fmod(b - a + 180, 360)
    if dif < 0:
        dif += 360
    return radians(dif - 180)

def trash_pursuit(timer):
    # Control gains
    Kp = 7

    # Stop if the path or state is invalid
    if desired_path is None or robot_state is None:
        ctrl.speed = 0
        ctrl.turn_angle = 0
    # If we are right next to the waypoint, just target it instead of a path
    # elif dist_to_goal < 5:#len(desired_path.poses) == 1:
    #     # TODO: can this speed up a bit?
    #     ctrl.speed = 2

    #     # Calculate desired heading difference
    #     dLat = goal.latitude - robot_state.latitude
    #     dLon = goal.longitude - robot_state.longitude
    #     desired_hdg = atan2(dLat, dLon)

    #     # Get the robot's heading
    #     hdg = robot_state.heading

    #     # Compute turn angle as a function of error
    #     error = angle_diff(desired_hdg, hdg)
    #     # print(hdg, desired_hdg, error)
    #     ctrl.turn_angle = Kp * error

    # Try to follow the path
    else:
        ctrl.speed = 1

        lookahead = 1 # meters

        # Get the target waypoint
        num_poses = len(desired_path.poses)
        lookahead_idx = int(lookahead / GRID_SIZE)

        # Dirty Pursuit
        if num_poses < lookahead_idx:
            lookahead_idx = num_poses - 1

        if lookahead_idx < 0 or lookahead_idx >= num_poses:
            return

        # Get next desired path point
        path_pt0 = desired_path.poses[0]
        look_pt = desired_path.poses[lookahead_idx]

        # If the robot is really far from the start of the path, wait for the path planner to catch up
        rdx = robot_state.x - path_pt0.pose.position.x
        rdy = robot_state.y - path_pt0.pose.position.y
        dist = rdx**2 + rdy**2
        if dist > 100:
            ctrl.speed = 0
            ctrl.turn_angle = 0
        # Otherwise follow the path normally
        else:
            # Calculate desired heading difference
            dx = look_pt.pose.position.x - path_pt0.pose.position.x
            dy = look_pt.pose.position.y - path_pt0.pose.position.y
            desired_hdg = atan2(dy, dx)

            # Get the robot's heading
            hdg = robot_state.heading

            # Compute turn angle as a function of error
            error = angle_diff(desired_hdg, hdg)

            # If error is greater than 10 degrees, drive slower until it isn't so bad
            if abs(error) > radians(10):
                ctrl.speed = 1

            # print(dx, dy, degrees(hdg), degrees(desired_hdg), degrees(error))
            ctrl.turn_angle = Kp * error

    # Publish the control needed
    ctrl_pub.publish(ctrl)



def setup():
    rospy.init_node("basic_control_node")

    # Subscribe to the path and the current state
    path_sub = rospy.Subscriber("/path", Path, path_callback, queue_size=1)
    pose_sub = rospy.Subscriber("/robot/state", RobotState, state_estimate_callback, queue_size=1)

    # Subscribe to the goal for close range alignment
    goal_sub = rospy.Subscriber("/task/goal", Gps, goal_callback, queue_size=1)

    # Try to follow the first part of the path at all times
    pursuit_timer = rospy.Timer(rospy.Duration(0.02), trash_pursuit, oneshot=False)

    # Pump callbacks
    rospy.spin()


if __name__ == "__main__":
    try:
        setup()
    except rospy.ROSInterruptException:
        pass