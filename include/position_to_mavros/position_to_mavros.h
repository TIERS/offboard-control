#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

class position_to_mavros
{
private:

    ros::NodeHandle nh_;

    ros::Publisher local_pos_pub_;

    ros::Subscriber uwb_sub_;

    ros::Subscriber lidar_sub_;

    ros::Subscriber vision_sub_; 

    ros::ServiceClient landing_client_;

    bool vision_only_;

    std::string pose_pub_topic_;
    
    std::string uwb_sub_topic_;

    std::string vision_sub_topic_;

    std::string lidar_sub_topic_;

public:
    geometry_msgs::PoseStamped uwb_pos_;

    double lidar_z_;

    geometry_msgs::PoseStamped vision_pos_;

    geometry_msgs::PoseStamped local_pos_;

    geometry_msgs::PoseStamped vision_last_pos_;


public:
    position_to_mavros(ros::NodeHandle& nh);
    ~position_to_mavros();


private:

    void uwb_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void lidar_callback(const sensor_msgs::Range::ConstPtr& msg);

    void vision_callback(const nav_msgs::Odometry::ConstPtr& msg);


public:
    void run(const ros::TimerEvent& event);
};