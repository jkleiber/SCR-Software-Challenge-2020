#!/usr/bin/env python2

import rospy

from math import radians

from sensor_msgs.msg import LaserScan

FOV = radians(10)       # field of view for obstructions in radians
OBSTRUCTION_DIST = 1.0  # Obstacle distance that causes a halt

# Is there an obstacle in the way
obstruction_alert = False


def in_front(angle):
    return angle < (FOV) and angle > (360 - FOV)


def laser_scan_callback(scan_data):
    global obstruction_alert

    # Find allowable ranges
    range_min = scan_data.range_min
    range_max = scan_data.range_max

    # Mark obstacles at different points based on the scan
    scan_idx = 0
    angle = scan_data.angle_min
    while angle <= scan_data.angle_max:

        # Find the range to an obstacle at this angle
        range_data = scan_data.ranges[scan_idx]

        # If the range is inside the boundaries, is in front of the robot, and the obstacle is close enough,
        # then prevent the robot from hitting it by remembering where it is
        if range_min <= range_data and range_data <= range_max and range_data < OBSTRUCTION_DIST:
            obstruction_alert = True
            return

    # No obstacles were in the view, so let the robot drive
    obstruction_alert = False
