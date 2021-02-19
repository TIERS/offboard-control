#include "position_to_mavros/position_to_mavros.h"
#include "safe_offboard/safe_offboard.h"

#include <yaml-cpp/yaml.h>
#include <ros/package.h>



int main(int argc, char **argv)
{

    ros::init(argc, argv, "offboard_node");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    double takeoff_height;
    double circle_radius;
    std::string flight_mode;
    double pos_valid_time;
    double waypoint_valid_time;
    std::vector<double> fly_fence{-1.0, -1.0, 0.0, 1.0, 1.0, 1.0};

    nh.param<double>("safe_offboard/takeoff_height", takeoff_height, 1.23);
    nh.param<double>("safe_offboard/circle_radius", circle_radius, 1.23);
    nh.param<std::string>("safe_offboard/flight_mode", flight_mode, std::string("stay"));
    nh.param<double>("safe_offboard/pos_valid_time", pos_valid_time, 1.0);
    nh.param<double>("safe_offboard/waypoint_valid_time", waypoint_valid_time, 1.0);
    nh.getParam("safe_offboard/fly_fence",fly_fence);

    // std::string offboard_params_file = ros::package::getPath("offboard_control") + "/config/offboard_params.yaml";
    // ROS_INFO_STREAM("^^^^^^^^^^^^^^^ offboard params file: " << offboard_params_file << " ^^^^^^^^^^^^^^");
    // YAML::Node offboard_params = YAML::LoadFile(offboard_params_file);

    // position_to_mavros pm = position_to_mavros(n, true);
    // ros::Timer timer_01 = n.createTimer(ros::Duration(0.02), &position_to_mavros::run, &pm);

    safe_offboard so = safe_offboard(n);
    so.set_flight_mode(flight_mode);
    so.set_takeoff_height(takeoff_height);
    so.set_circle_radius(circle_radius);
    so.set_lower_fly_fence(fly_fence[0], fly_fence[1], fly_fence[2]);
    so.set_upper_fly_fence(fly_fence[3], fly_fence[4], fly_fence[5]);
    ros::Timer timer = n.createTimer(ros::Duration(0.05), &safe_offboard::check_poses, &so);
    so.run();
    
    ros::spin();
    return 0;
}
