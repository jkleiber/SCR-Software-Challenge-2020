#!/usr/bin/env python2

import rospy
import copy

from math import sin, cos, pi, fmod

import tf

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from swc_msgs.msg import RobotState

# Map size and resolution
MAP_RES = 0.1
MAP_WIDTH = int(60 / MAP_RES)   # 60 meters wide
MAP_HEIGHT = int(125 / MAP_RES) # 120 meters long

# LiDAR params
LIDAR_RANGE = 10

# Create the map
map_orientation = tf.transformations.quaternion_from_euler(0, 0, -pi/2)
map_origin = Pose()
map_origin.position = Point(x=0, y=MAP_WIDTH*MAP_RES, z=0)
map_origin.orientation = Quaternion(x=map_orientation[0], y=map_orientation[1], z=map_orientation[2], w=map_orientation[3])
map_info = MapMetaData(resolution=MAP_RES, width = MAP_WIDTH, height = MAP_HEIGHT, origin=map_origin)
map_data = OccupancyGrid(info = map_info, data = [0] * (MAP_HEIGHT * MAP_WIDTH))
working_map_data = [0] * (MAP_HEIGHT * MAP_WIDTH)

# Set up a local map frame to work inside
# HACK: this will cut parts off of the global map because the local map isn't a circle.
local_map_size = 2*int(LIDAR_RANGE / MAP_RES)
local_map_center = int(local_map_size / 2)
local_map_info = MapMetaData(resolution=MAP_RES, width=local_map_size, height=local_map_size)
local_map_data = OccupancyGrid(info=local_map_info, data = [0] * local_map_size * local_map_size)
working_local_map = [0] * local_map_size * local_map_size

# Publisher for the maps
map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
local_map_pub = rospy.Publisher("/local_map", OccupancyGrid, queue_size=1)

# Robot information
ROBOT_WIDTH = 0.4
ROBOT_LENGTH = 0.8

# Track the robot's pose
robot_x = 0
robot_y = 0
robot_hdg = 0

# TODO: Consider adding a fading function to the identified landmarks (i.e. slowly reduce their value based on the number of times they've been seen)


def state_estimate_callback(state):
    """ Get the pose from the state variables """
    global robot_x, robot_y, robot_hdg, local_map_data

    # set the pose
    robot_x = state.x
    robot_y = state.y
    robot_hdg = state.heading

    ### update the local map's origin: position and orientation
    # Position
    local_map_data.info.origin.position = Point(x=0, y=0, z=0)

    # Orientation
    # always offset by pi/2
    # q = tf.transformations.quaternion_from_euler(0, 0, pi/2)
    # local_map_data.info.origin.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
    # NOTE: The map is always off by pi/2, but this is solved in the below transformation
    local_map_data.info.origin.orientation = Quaternion()

    # Find heading with the pi/2 map rotation offset included
    hdg = fmod(robot_hdg + pi/2, 2*pi)

    ### In order to make the local map display nicely in RViz, need to apply a rotation as the robot rotates
    ### Link: https://stackoverflow.com/questions/20936429/rotating-a-rectangle-shaped-polygon-around-its-center-java
    # Find anchor point (the origin position of the map) for local map during 0 degree heading
    anchor_x = robot_x - LIDAR_RANGE
    anchor_y = robot_y - LIDAR_RANGE

    # Translate 0 degree rectangle to origin (robot position is the center)
    origin_x = anchor_x - robot_x
    origin_y = anchor_y - robot_y

    # Apply rotation matrix
    rot_x = origin_x * cos(hdg) - origin_y * sin(hdg)
    rot_y = origin_x * sin(hdg) + origin_y * cos(hdg)

    # Translate rotated rectangle back to robot center
    local_map_x = robot_x + rot_x
    local_map_y = robot_y + rot_y



    # Broadcast the new transform between the robot and the map
    broadcaster = tf.TransformBroadcaster()
    # broadcaster.sendTransform((0, 0, 0),
    #                           tf.transformations.quaternion_from_euler(0, 0, 0),
    #                           rospy.Time.now(),
    #                           "base_link",
    #                           "map")
    # broadcaster.sendTransform((60, 0, 0),
    #                           tf.transformations.quaternion_from_euler(0, 0, pi/2),
    #                           rospy.Time.now(),
    #                           "base_link",
    #                           "map")
    broadcaster.sendTransform((0,0,0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "base_link",
                              "map")
    broadcaster.sendTransform((local_map_x, local_map_y, 0),
                              tf.transformations.quaternion_from_euler(0, 0, hdg),
                              rospy.Time.now(),
                              "local_map",
                              "map")


def laser_scan_callback(scan_data):
    global working_map_data, working_local_map

    # Update working_map_data
    working_map_data = copy.copy(map_data.data)
    working_local_map = [0] * (local_map_size**2)

    # Find allowable ranges
    range_min = scan_data.range_min
    range_max = scan_data.range_max

    # Mark obstacles at different points based on the scan
    scan_idx = 0
    angle = scan_data.angle_min
    while angle <= scan_data.angle_max:

        # Find the range to an obstacle at this angle
        range_data = scan_data.ranges[scan_idx]

        # If the range is inside the boundaries, then consider it
        if range_min <= range_data and range_data <= range_max:
            # Find the x and y offset in meters
            x_offset = range_data * cos(angle)
            y_offset = range_data * sin(angle)

            # Bin the offsets on the local map frame
            x_bin = local_map_center - int(x_offset / MAP_RES)
            y_bin = local_map_center - int(y_offset / MAP_RES)

            # Update the local map appropriately
            index = x_bin*local_map_size + y_bin
            if index >= 0 and index < (local_map_size**2):
                working_local_map[index] = 100

        # Increment the angle and scan index
        angle += scan_data.angle_increment
        scan_idx += 1

    #### Add the local map frame to the working global map
    # Find top-left corner of local frame
    start_x = int((robot_x + LIDAR_RANGE) / MAP_RES)
    start_y = int((robot_y + LIDAR_RANGE) / MAP_RES)

    # Set the global map based on the local map frame
    for i in range(0, len(working_local_map)):
        local_x = start_x - int(i / local_map_size)
        local_y = start_y - int(i % local_map_size)

        idx = local_x*MAP_WIDTH + local_y
        if idx >=0 and idx < (MAP_WIDTH * MAP_HEIGHT):
            working_map_data[idx] = working_local_map[i]

    # Update the current local map from the working layer
    tmp_local_map = local_map_data.data
    local_map_data.data = working_local_map
    working_local_map = tmp_local_map

    # Update the current map data from the working layer
    tmp_data = map_data.data
    map_data.data = working_map_data
    working_map_data = tmp_data



def get_configuration_space(timer_event):
    """ Go through the map and inflate obstacles so the robot doesn't hit them """
    WIDTH_INFLATE_FACTOR = int(ROBOT_WIDTH / MAP_RES)
    HEIGHT_INFLATE_FACTOR = int(ROBOT_LENGTH / MAP_RES)

    # Get a deep copy of the map for the configuration space map
    config_space = copy.deepcopy(map_data)
    working_config_space = copy.copy(config_space.data)

    # Go through each point on the map to see if it needs inflation
    for row in range(0, MAP_HEIGHT):
        for col in range(0, MAP_WIDTH):
            # Get the cost of the obstacle in the active configuration space
            cost = config_space.data[row * MAP_WIDTH + col]

            # If the cost is non-zero, inflate the obstacle at its current cost
            if cost > 0:
                for i in range(row - HEIGHT_INFLATE_FACTOR, row + HEIGHT_INFLATE_FACTOR + 1):
                    for j in range(col - WIDTH_INFLATE_FACTOR, col + WIDTH_INFLATE_FACTOR + 1):
                        index = i*MAP_WIDTH + j

                        # Only update the index if it's in range
                        if index >= 0 and index < (MAP_HEIGHT * MAP_WIDTH):
                            # Update the working configuration space
                            working_config_space[index] = cost

    # Update header
    config_space.header.stamp = rospy.Time.now()
    config_space.header.frame_id = "map"

    # Publish the updated configuration space
    config_space.data = working_config_space
    map_pub.publish(config_space)


def get_local_configuration_space(timer_event):
    """ Go through the map and inflate obstacles so the robot doesn't hit them """
    WIDTH_INFLATE_FACTOR = int(ROBOT_WIDTH*2 / MAP_RES)
    HEIGHT_INFLATE_FACTOR = int(ROBOT_LENGTH*2 / MAP_RES)

    # Get a deep copy of the map for the configuration space map
    local_config_space = copy.deepcopy(local_map_data)
    working_local_config_space = copy.copy(local_config_space.data)

    # Go through each point on the map to see if it needs inflation
    for row in range(0, local_map_size):
        for col in range(0, local_map_size):
            # Get the cost of the obstacle in the active configuration space
            cost = local_config_space.data[row * local_map_size + col]

            # If the cost is non-zero, inflate the obstacle at its current cost
            if cost > 0:
                for i in range(row - HEIGHT_INFLATE_FACTOR, row + HEIGHT_INFLATE_FACTOR + 1):
                    for j in range(col - WIDTH_INFLATE_FACTOR, col + WIDTH_INFLATE_FACTOR + 1):
                        index = i*local_map_size + j

                        # Only update the index if it's in range
                        if index >= 0 and index < (local_map_size * local_map_size):
                            # Update the working configuration space
                            # Get distance from obstacle
                            dist = (i-row)**2 + (j-col)**2
                            # If this is an "inner circle," set to max cost
                            if dist < (max(WIDTH_INFLATE_FACTOR, HEIGHT_INFLATE_FACTOR) / 2)**2:
                                working_local_config_space[index] = cost
                            else:
                                working_local_config_space[index] = int(cost / 2)


    # DEBUG: check to see where the origin is in RViz
    # working_local_config_space[0] = 100

    # Update header
    local_config_space.header.stamp = rospy.Time.now()
    local_config_space.header.frame_id = "local_map"

    # Publish the updated configuration space
    local_config_space.data = working_local_config_space
    local_map_pub.publish(local_config_space)



if __name__ == "__main__":
    # Initialize node
    rospy.init_node("map_manager_node")

    # Subscribe to the localization system and lidar
    lidar_sub = rospy.Subscriber("/scan", LaserScan, laser_scan_callback, queue_size=1)
    state_sub = rospy.Subscriber("/robot/state", RobotState, state_estimate_callback, queue_size=1)

    # Publish the timer on an interval
    map_timer = rospy.Timer(rospy.Duration(0.1), get_configuration_space)
    local_map_timer = rospy.Timer(rospy.Duration(0.1), get_local_configuration_space)

    # Run the node and callbacks
    rospy.spin()
