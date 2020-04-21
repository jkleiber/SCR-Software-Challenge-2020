#!/usr/bin/env python2

import rospy

from math import radians, pi, degrees, fmod

from sensor_msgs.msg import LaserScan
from swc_msgs.msg import Control

FOV = radians(20)       # field of view for obstructions in radians
OBSTRUCTION_DIST = 2.5  # Obstacle distance that causes a halt

# Is there an obstacle in the way
obstruction_alert = False

# Control publisher
ctrl_pub = rospy.Publisher("/sim/control", Control, queue_size=1)


def angle_diff(angle1, angle2):
    a = degrees(angle1)
    b = degrees(angle2)

    dif = fmod(b - a + 180, 360)
    if dif < 0:
        dif += 360
    return radians(dif - 180)

def in_front(angle):
    return angle_diff(angle, 0) <= FOV


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
        if range_min <= range_data and range_data <= range_max and range_data <= OBSTRUCTION_DIST and in_front(angle):
            obstruction_alert = True
            return

        angle += scan_data.angle_increment
        scan_idx += 1

    # No obstacles were in the view, so let the robot drive
    obstruction_alert = False


def control_callback(ctrl):
    # If the robot is going forward and there is an obstacle, slow down
    if obstruction_alert and ctrl.speed > 0 and abs(ctrl.turn_angle) < radians(20):
        ctrl.speed = 0.1 * ctrl.speed
        print("OBSTRUCTION: Slowing robot down!")

    ctrl_pub.publish(ctrl)


def obstacle_node():
    rospy.init_node("obstacle_node")

    # Get raw laser scan data
    laser_sub = rospy.Subscriber("/scan", LaserScan, laser_scan_callback, queue_size=1)

    # Subscribe to the controller output
    ctrl_sub = rospy.Subscriber("kleiber/control", Control, control_callback, queue_size=1)

    rospy.spin()

# Main setup
if __name__ == '__main__':
    try:
        obstacle_node()
    except rospy.ROSInterruptException:
        pass
