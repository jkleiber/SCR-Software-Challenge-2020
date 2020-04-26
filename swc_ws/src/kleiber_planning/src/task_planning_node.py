#!/usr/bin/env python2

import rospy
import time

from gps_calc import measure_gps
from swc_msgs.msg import Gps, RobotState
from swc_msgs.srv import Waypoints

# Waypoint trackers
wpt_list = None
active_wpt = None
active_wpt_idx = 0

# Distance from GPS fix to proceed to next target
# HACK: aiming for 1 meter accuracy
DIST_EPS = 2.5  # meter

# Waypoint publisher
wpt_pub = rospy.Publisher("/task/goal", Gps, queue_size=1)




def state_estimate_callback(state):
    """ Get the GPS coordinates from the state variables """
    global active_wpt, active_wpt_idx

    # Wait for the active waypoint to be selected first
    if active_wpt is None:
        return

    # Get current GPS
    lat = state.latitude
    lon = state.longitude

    # Find the squared distance between this and the active waypoint in meters
    dist = measure_gps(lat, lon, active_wpt.latitude, active_wpt.longitude)

    # if the distance is close enough, then proceed to the next waypoint
    if dist < DIST_EPS:
        active_wpt_idx += 1

        # If there is another waypoint in the list, set it:
        if active_wpt_idx < len(wpt_list):
            active_wpt = wpt_list[active_wpt_idx]
            print("NEW WAYPOINT: ", active_wpt)

            # publish active waypoint
            wpt_pub.publish(active_wpt)




def main_loop():
    global wpt_list, active_wpt, active_wpt_idx
    rospy.init_node("task_planning_node")

    # Subscribe to localization updates
    state_sub = rospy.Subscriber("/robot/state", RobotState, state_estimate_callback)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()

    # Make the active waypoint list
    wpt_list = waypoints.waypoints
    active_wpt_idx = 0
    active_wpt = wpt_list[active_wpt_idx]
    
    # Wait for task/goal to have a subscriber
    while wpt_pub.get_num_connections() < 2:
        time.sleep(0.1)
    
    # Planning ready
    print("Task Planning Ready")

    rospy.spin()


if __name__ == "__main__":
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
