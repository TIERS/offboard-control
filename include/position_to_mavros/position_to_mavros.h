#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class position_to_mavros
{
private:

    ros::NodeHandle nh_;

    ros::Publisher local_pos_pub;

    ros::Subscriber uwb_sub;

    ros::Subscriber lidar_sub;

    ros::Subscriber vision_sub; 

public:
    geometry_msgs::PoseStamped uwb_pos;

    geometry_msgs::PoseStamped lidar_pos;

    double lidar_z;

    geometry_msgs::PoseStamped vision_pos;

    geometry_msgs::PoseStamped local_pos;


public:
    position_to_mavros(ros::NodeHandle& nh, bool vision_only=true);
    ~position_to_mavros();


private:

    void uwb_callback(const geometry_msgs::Pose::ConstPtr& msg);

    void lidar_callback(const sensor_msgs::Range::ConstPtr& msg);

    void vision_callback(const nav_msgs::Odometry::ConstPtr& msg);


public:
    void run(const ros::TimerEvent& event);
};