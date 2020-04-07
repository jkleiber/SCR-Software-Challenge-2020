#!/usr/bin/env python

import rospy
import math
from swc_msgs.msg import Control
from swc_msgs.srv import Waypoints
from geometry_msgs.msg import Twist

_control_pub = None
control_msg = Control()

def keyboard_callback(twist):
    global control_msg

    control_msg.speed = twist.linear.x
    control_msg.turn_angle = twist.angular.z

    # Publish the message to /sim/control so the simulator receives it
    _control_pub.publish(control_msg)


def main():
    global _control_pub

    # Initalize our node in ROS
    rospy.init_node('manual_control_node')

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Subscribe to the keyboard
    keyboard_sub = rospy.Subscriber("/cmd_vel", Twist, keyboard_callback, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()

    print(waypoints.waypoints)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
