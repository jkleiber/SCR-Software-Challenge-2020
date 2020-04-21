#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <swc_msgs/Gps.h>
#include <swc_msgs/Control.h>
#include <swc_msgs/RobotState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>


// State variables
double latitude;
double longitude;
double x;
double y;
double heading;
double velocity;
double turn_angle;
double angular_accel;
double linear_accel;

// Publishers
ros::Publisher state_pub;
ros::Publisher pose_pub;


/**
 * @brief GPS sensor callback
 *
 * @param gps_msg
 */
void gpsCallback(const swc_msgs::Gps::ConstPtr& gps_msg)
{
    // std::cout << std::setprecision(10) << gps_msg->latitude << ", " << gps_msg->longitude << std::endl;

    // Set latitude and longitude
    latitude = gps_msg->latitude;
    longitude = gps_msg->longitude;
}


/**
 * @brief IMU sensor callback
 *
 * @param imu_msg
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // Get heading from quaternion
    heading = tf::getYaw(imu_msg->orientation);

    // Get linear and angular velocity
    linear_accel = imu_msg->linear_acceleration.x;
    angular_accel = imu_msg->angular_velocity.z;
}


/**
 * @brief Intercept controls sent to the robot to get turn angle estimates
 *
 * @param ctrl_msg
 */
void controlCallback(const swc_msgs::Control::ConstPtr& ctrl_msg)
{
    turn_angle = ctrl_msg->turn_angle;
}


/**
 * @brief Velocity sensor callback
 *
 * @param vel_msg
 */
void velocityCallback(const std_msgs::Float32::ConstPtr& vel_msg)
{
    velocity = vel_msg->data;
}



void statePublisher(const ros::TimerEvent& timer)
{
    swc_msgs::RobotState state_msg;

    // Dead-reckoning
    double dt = (timer.current_real - timer.last_real).toSec();
    x = x + velocity * cos(heading) * dt;
    y = y + velocity * sin(heading) * dt;

    state_msg.latitude = latitude;
    state_msg.longitude = longitude;
    state_msg.x = x;
    state_msg.y = y;
    state_msg.heading = heading;
    state_msg.velocity = velocity;
    state_msg.turn_angle = turn_angle;
    state_msg.linear_acceleration = linear_accel;
    state_msg.angular_acceleration = angular_accel;

    // Publish the state
    state_pub.publish(state_msg);
}


void posePublisher(const ros::TimerEvent& timer)
{
    geometry_msgs::PoseStamped robot_pose;

    // Stamp
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = "map";

    // Set position
    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    robot_pose.pose.position.z = 0;

    // Set orientation
    tf::Quaternion q = tf::createQuaternionFromYaw(heading);
    robot_pose.pose.orientation.w = q.getW();
    robot_pose.pose.orientation.x = q.getX();
    robot_pose.pose.orientation.y = q.getY();
    robot_pose.pose.orientation.z = q.getZ();

    // Publish
    pose_pub.publish(robot_pose);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");

    ros::NodeHandle loc_node;

    // Read initialization params
    latitude = loc_node.param("localization_node/lat", 0);
    longitude = loc_node.param("localization_node/lon", 0);
    x = loc_node.param("localization_node/x", 0);
    y = loc_node.param("localization_node/y", 0);
    heading = loc_node.param("localization_node/hdg", 0);
    velocity = loc_node.param("localization_node/vel", 0);
    turn_angle = loc_node.param("localization_node/turn_angle", 0);
    linear_accel = loc_node.param("localization_node/linear_accel", 0);
    angular_accel = loc_node.param("localization_node/angular_accel", 0);

    // Subscribe to all sensor readings
    ros::Subscriber gps_sub = loc_node.subscribe(loc_node.resolveName("/sim/gps"), 1, &gpsCallback);
    ros::Subscriber imu_sub = loc_node.subscribe(loc_node.resolveName("/sim/imu"), 1, &imuCallback);
    ros::Subscriber vel_sub = loc_node.subscribe(loc_node.resolveName("/sim/velocity"), 1, &velocityCallback);

    // Subscribe to the control publications
    ros::Subscriber ctrl_sub = loc_node.subscribe(loc_node.resolveName("/sim/control"), 1, &controlCallback);

    // Advertise the state and pose publishers
    state_pub = loc_node.advertise<swc_msgs::RobotState>(loc_node.resolveName("/robot/state"), 1);
    pose_pub = loc_node.advertise<geometry_msgs::PoseStamped>(loc_node.resolveName("/robot/pose"), 1);

    // Publish the robot's state/pose periodically
    ros::Timer state_pub_timer = loc_node.createTimer(ros::Duration(0.05), &statePublisher);
    ros::Timer pose_pub_timer = loc_node.createTimer(ros::Duration(0.05), &posePublisher);

    ros::spin();

    return 0;
}
