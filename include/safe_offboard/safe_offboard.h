#pragma once
#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include "offboard_control/flight_mode.h"
#include "offboard_control/offboard_state.h"

struct point
{
    double x;
    double y;
    double z;
};

struct fly_fence
{
   point lower_point;
   point upper_point;
};


class safe_offboard
{
private:

    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;

    ros::Subscriber position_cb_;

    ros::Subscriber flight_mode_sub_;

    ros::Subscriber external_waypoint_cb_;

    ros::Publisher waypoint_pub_;

    ros::Publisher flight_mode_pub_;

    ros::Publisher offboard_state_pub_;

    ros::ServiceClient arming_client_;

    ros::ServiceClient set_mode_client_;

    ros::ServiceServer emergency_land_server_;

    ros::ServiceServer flight_mode_srv_;

    ros::ServiceServer offboard_state_srv_;

private:
    mavros_msgs::State current_state_;

    geometry_msgs::PoseStamped current_pos_;

    geometry_msgs::PoseStamped current_objective_;

    geometry_msgs::PoseStamped next_external_waypoint_; 

    geometry_msgs::PoseStamped next_waypoint_; 

    geometry_msgs::PoseStamped home_pos_;

    mavros_msgs::SetMode offb_set_mode_;

    mavros_msgs::CommandBool arm_cmd_;

    std::string flight_mode_;

    double takeoff_height_;

    double circle_radius_;

    fly_fence fly_fence_;

    double pos_valid_time_;

    double waypoint_valid_time_;

    bool emergency_landing_;

    bool taken_off_;

    std::string offboard_state_;

public:
    safe_offboard(ros::NodeHandle& nh);
    ~safe_offboard();

public:
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    void mode_cb(const std_msgs::String::ConstPtr& msg);

    bool flight_mode_srv_cb(offboard_control::flight_mode::Request &request, offboard_control::flight_mode::Response &response);

    bool offboard_state_srv_cb(offboard_control::offboard_state::Request &request, offboard_control::offboard_state::Response &response);

    bool emergency_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

    bool check_ext_position();

    void check_poses(const ros::TimerEvent& event);

    bool check_inside_fly_fence(double x, double y, double z);

    void update_current_pos(const geometry_msgs::PoseStamped::ConstPtr& pose);

    void update_external_waypoint(const geometry_msgs::PoseStamped::ConstPtr& pose);

    void update_current_objective();

    void update_current_objective(geometry_msgs::PoseStamped& objective);

    void emergency_land();

    double pose_to_pose_dist(const geometry_msgs::PoseStamped& pose1,
                            const geometry_msgs::PoseStamped& pose2);

    void set_flight_mode(std::string flight_mode);

    void set_takeoff_height(double takeoff_height);

    void set_circle_radius(double circle_radius); 

    void set_home_pos(geometry_msgs::PoseStamped& pose);    

    void set_lower_fly_fence(double x, double y, double z);

    void set_upper_fly_fence(double x, double y, double z);  

    void pub_offboard_state(const ros::TimerEvent& event);

    void run(); 

};

