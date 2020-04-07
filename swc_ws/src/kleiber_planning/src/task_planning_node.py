#!/usr/bin/env python2

import rospy
from math import sin, cos, atan2, radians, pi, sqrt

from swc_msgs.msg import Gps, RobotState
from swc_msgs.srv import Waypoints

# Waypoint trackers
wpt_list = None
active_wpt = None
active_wpt_idx = 0

# Distance from GPS fix to proceed to next target
DIST_EPS = 1.0  # meter

# Waypoint publisher
wpt_pub = rospy.Publisher("/task/goal", Gps, queue_size=1)

def measure_gps(lat1, lon1, lat2, lon2):
    """ Haversine formula for finding distance between two GPS points """
    R = 6378.137; # Radius of earth in KM
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)
    a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c;
    return d * 1000; # meters


def state_estimate_callback(state):
    """ Get the GPS coordinates from the state variables """
    global active_wpt, active_wpt_idx

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

    rospy.spin()


if __name__ == "__main__":
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass