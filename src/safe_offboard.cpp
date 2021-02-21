#include "safe_offboard/safe_offboard.h"
#include <math.h>


safe_offboard::safe_offboard(ros::NodeHandle& nh)
{
    nh_ = nh;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    emergency_landing_ = false;
    
    current_pos_.pose.position.x = -1;
    current_pos_.pose.position.y = -1;
    current_pos_.pose.position.z = -1;

    flight_mode_ = "stay";
    takeoff_height_ = 1.0;
    circle_radius_ = 1.0;
    fly_fence_.lower_point = point{-2.0,-2.0, 0.0};
    fly_fence_.upper_point = point{ 2.0, 2.0, 2.0};
    pos_valid_time_ = 1.0;
    waypoint_valid_time_ = 1.0;
    emergency_landing_ = false;
    taken_off_ = false;

    offboard_state_ = "disarmed"; // ["disarmed", "armed", "taking_off", "flying", "landing", "emergency", "going_home"]


    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &safe_offboard::state_cb, this);
    position_cb_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10, &safe_offboard::update_current_pos, this);
    external_waypoint_cb_ = nh_.subscribe<geometry_msgs::PoseStamped>("offboard/command_waypoint", 10, &safe_offboard::update_external_waypoint, this);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    emergency_land_server_ = nh_.advertiseService("offboard/emergency_land", &safe_offboard::emergency_srv_cb, this);
    flight_mode_sub_ = nh_.subscribe<std_msgs::String>("offboard/mode", 10, &safe_offboard::mode_cb, this);
}

safe_offboard::~safe_offboard()
{
}

void safe_offboard::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
    if ( !current_state_.armed ) {
        offboard_state_ = "disarmed";
    }
    if ( current_state_.armed && offboard_state_ == "disarmed") {
        offboard_state_ = "armed";
        // std::cout << " --------> AAAAARMEEEED " << std::endl;
    }
}

void safe_offboard::mode_cb(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "land")
    {
        flight_mode_ = "land";
    }
    if (msg->data == "stay")
    {
        flight_mode_ = "stay";
    }
    if (msg->data == "hover")
    {
        flight_mode_ = "hover";
    }
    if (msg->data == "circle")
    {
        flight_mode_ = "circle";
    }
    if (msg->data == "external_control")
    {
        flight_mode_ = "external_control";
    }
    
}

 bool safe_offboard::emergency_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response){
//	try{
		if (request.data)
		{
		    emergency_landing_ = true;
		}
		else
		{
		    emergency_landing_ = false;
		}
		response.success = true;
//	}
 
//	catch(...){
//		ROS_WARN("Impossible to execute Start-Move service");
//		ros::Duration(1.0).sleep();
//		response.success = false;
//		//continue;
//        }
 
        ROS_INFO_STREAM("Emergecy Landing Request: " <<  request.data );
        ROS_INFO_STREAM("Emergecy Landing Sending Back Response: " << response.success); 
        return true;
 }

bool safe_offboard::check_ext_position(){
    if (current_pos_.pose.position.x == -1 && 
        current_pos_.pose.position.y == -1 &&
        current_pos_.pose.position.z == -1)
    {
        ROS_ERROR_STREAM("Check If the External Positioning Is Working");
        return false;
    }
    home_pos_ = current_pos_;
}

bool safe_offboard::check_inside_fly_fence(double x, double y, double z){
    bool lower_flag = (x-fly_fence_.lower_point.x) >= 0 &&
                      (y-fly_fence_.lower_point.y) >= 0 && 
                      (z-fly_fence_.lower_point.z) >= 0;
    bool upper_flag = (x-fly_fence_.upper_point.x) <= 0 && 
                      (y-fly_fence_.upper_point.y) <= 0 && 
                      (z-fly_fence_.upper_point.z) <= 0;
    return lower_flag && upper_flag;
}

 void safe_offboard::emergency_land(){

 }

double safe_offboard::pose_to_pose_dist(const geometry_msgs::PoseStamped& pose1,
                        const geometry_msgs::PoseStamped& pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    return std::pow(dx * dx + dy * dy + dz * dz, 0.5);
}


void safe_offboard::update_current_pos(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    current_pos_ = *pose;
}

void safe_offboard::update_external_waypoint(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    next_external_waypoint_ = *pose;
}

void safe_offboard::update_current_objective(){
    next_waypoint_ = home_pos_;

    // std::cout << "Offboard state: " << offboard_state_ << std::endl;
    // std::cout << "Flight mode: " << flight_mode_ << std::endl;

    if (!current_state_.armed) 
    {
        taken_off_ = false;
        next_waypoint_.header.stamp = ros::Time::now();
        if (offboard_state_ != "disarmed" && offboard_state_ != "landing" && offboard_state_ != "emergency")
        {
            offboard_state_ = "disarmed";
            ROS_INFO_STREAM(">>>>>> MAV IS NOT YET ARMED <<<<<<");
        }
        // do nothing (already set to home pos)
    }
    else if (current_state_.armed && !taken_off_)
    {
        if (offboard_state_ != "taking_off" && offboard_state_ != "landing" && offboard_state_ != "emergency")
        {
            offboard_state_ = "taking_off";
            ROS_INFO_STREAM(">>>>>> MAV IS TAKING OFF <<<<<<");
        }

        if (offboard_state_ == "taking_off")
        {
        
            next_waypoint_.pose.position.z = takeoff_height_;
            next_waypoint_.header.stamp = ros::Time::now();
            
            if (current_pos_.pose.position.z > takeoff_height_ - 0.2) {
                taken_off_ = true;
                offboard_state_ = "flying";
                ROS_INFO_STREAM(">>>>>> MAV IS UP IN THE SKY -- READY TO FLY <<<<<<");
            }
        }
        
    }
    else 
    {
        if (flight_mode_ == "land" || offboard_state_ == "landing")
        {
            next_waypoint_ = current_pos_;
            next_waypoint_.pose.position.z = 0;
            next_waypoint_.header.stamp = ros::Time::now();
        }
        else if (offboard_state_ == "going_home")
        {
            next_waypoint_ = home_pos_;
            next_waypoint_.pose.position.z = 0;
            next_waypoint_.header.stamp = ros::Time::now();
        }
        else if (flight_mode_ == "stay" && offboard_state_ == "flying")
        {
            next_waypoint_.pose.position.z = takeoff_height_;
            next_waypoint_.header.stamp = ros::Time::now();
        }
        else if (flight_mode_ == "hover" && offboard_state_ == "flying")
        {
            next_waypoint_ = current_pos_;
            next_waypoint_.header.stamp = ros::Time::now();
        }
        else if (flight_mode_ == "circle" && offboard_state_ == "flying")
        {
            double theta = atan2(current_pos_.pose.position.y, current_pos_.pose.position.x);
            double new_theta = fmod(theta + 0.5, 2*M_PI);
            next_waypoint_.pose.position.x = cos(new_theta)*circle_radius_;
            next_waypoint_.pose.position.y = sin(new_theta)*circle_radius_;
            next_waypoint_.pose.position.z = takeoff_height_;

            next_waypoint_.pose.orientation.x = 0;
            next_waypoint_.pose.orientation.y = 0;
            next_waypoint_.pose.orientation.z = 0;
            next_waypoint_.pose.orientation.w = 1;
            next_waypoint_.header.stamp = ros::Time::now();      
        }
        else if (flight_mode_ == "external_control" && offboard_state_ == "flying")
        {
            next_waypoint_ = next_external_waypoint_;
        }
    }

    // std::cout << " --> " << offboard_state_ << std::endl;
    // std::cout << " Next waypoint: " << next_waypoint_.pose.position.x << ", " << next_waypoint_.pose.position.y << ", " << next_waypoint_.pose.position.z << ", " << std::endl;
}

void safe_offboard::update_current_objective(geometry_msgs::PoseStamped& objective){
    next_external_waypoint_ = objective;
}

void safe_offboard::set_flight_mode(std::string flight_mode)
{
    flight_mode_ = flight_mode;
    ROS_INFO_STREAM("Set Flight Mode Is " << flight_mode_);
}

void safe_offboard::set_takeoff_height(double takeoff_height)
{
    takeoff_height_ = takeoff_height;
    ROS_INFO_STREAM("Set Takeoff Height Is " << takeoff_height_);
}

void safe_offboard::set_circle_radius(double circle_radius)
{
    circle_radius_ = circle_radius;
    ROS_INFO_STREAM("Set Circle Radius Is " << circle_radius_);
} 

void safe_offboard::set_home_pos(geometry_msgs::PoseStamped& pose)
{
    home_pos_ = pose;
    ROS_INFO_STREAM("Home Position Is (" << pose.pose.position.x << " , "
                    << pose.pose.position.y << " , "
                    << pose.pose.position.z << ")");
}


void safe_offboard::set_lower_fly_fence(double x, double y, double z)
{
    fly_fence_.lower_point.x = x;
    fly_fence_.lower_point.y = y;
    fly_fence_.lower_point.z = z;
    ROS_INFO_STREAM("Set Lower Fly Fence to (" << fly_fence_.lower_point.x << " , "
                << fly_fence_.lower_point.y << " , "
                << fly_fence_.lower_point.z << ")");
}

void safe_offboard::set_upper_fly_fence(double x, double y, double z)
{
    fly_fence_.upper_point.x = x;
    fly_fence_.upper_point.y = y;
    fly_fence_.upper_point.z = z;
    ROS_INFO_STREAM("Set Upper Fly Fence to (" << fly_fence_.upper_point.x << " , "
            << fly_fence_.upper_point.y << " , "
            << fly_fence_.upper_point.z << ")");
} 


void safe_offboard::check_poses(const ros::TimerEvent& event)
{
    update_current_objective();

    //check the position timestamps //TODO: Change the topic to vio
    if((ros::Time::now() - current_pos_.header.stamp) > ros::Duration(pos_valid_time_)) 
    {
        //Land if timestamp is very old
        ROS_INFO_STREAM(">>>>>> Out of Position Valid Time <<<<<<");
        next_waypoint_.pose.position.z = 0.0;
        offboard_state_ = "landing";
    }
    //check the waypoint is old or not
    else if((ros::Time::now() - next_waypoint_.header.stamp) > ros::Duration(waypoint_valid_time_)) 
    {
        //Home if it is very old
        ROS_INFO_STREAM(">>>>>> Out of Waypoint Valid Time <<<<<<");
        next_waypoint_.pose.position.z = 0.0;
        offboard_state_ = "landing";
    }
    // check the waypoint is out of range or not
    else if(!check_inside_fly_fence(next_waypoint_.pose.position.x, 
        next_waypoint_.pose.position.y, next_waypoint_.pose.position.z)) 
    {
        ROS_INFO_STREAM(">>>>>> Out of Fly Fence, Next Waypoint Is " << 
                        next_waypoint_.pose.position.x << " , " <<
                        next_waypoint_.pose.position.y << " , " <<
                        next_waypoint_.pose.position.z << " <<<<<<");
        next_waypoint_.pose.position.z = 0.0; 
        offboard_state_ = "landing";
    }
    waypoint_pub_.publish(next_waypoint_);
}


void safe_offboard::run(){
    //the setpoint publishing rate MUST be faster than 2Hz required by offboard mode
    ros::Rate rate(20.0);
    
    // wait for FCU connection
    // ROS_DEBUG_STREAM();
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    for (size_t i = 0; i < 80; i++)
    {
        if(current_state_.connected)
            break;
        else
            rate.sleep();
    }
    if(!current_state_.connected){
        ROS_ERROR_STREAM("Error! Could not connect to FCU... Exiting now.");
    }
        
    // Send a few waypoints before starting (just the current position
    ROS_DEBUG_STREAM("Sending a few waypoints before starting (2 seconds)...");
    for(int i = 40; ros::ok() && i > 0; --i){
        waypoint_pub_.publish(current_pos_);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
		// if( current_state_.mode != "OFFBOARD" &&
		// 		(ros::Time::now() - last_request > ros::Duration(2.0))){
		// 		if( set_mode_client_.call(offb_set_mode_) &&
		// 				offb_set_mode_.response.mode_sent){
		// 				ROS_INFO_STREAM("Offboard Enabled");
		// 		}
		// 		last_request = ros::Time::now();
		// } else {
		// 		if( !current_state_.armed &&
		// 				(ros::Time::now() - last_request > ros::Duration(2.0))){
		// 				if( arming_client_.call(arm_cmd_) &&
		// 						arm_cmd_.response.success){
		// 						ROS_INFO_STREAM("Vehicle Armed");
		// 				}
		// 				last_request = ros::Time::now();
		// 		}
        //         // else if(current_state_.armed){
                    
        //         // }
		// }
        // 
        ros::spinOnce();
		rate.sleep();
    }


    // for(size_t i = 0; i < 10; i++){
    //     // land*
    //     safe_offboard::safe_offboard::::p(0.1);
    // }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "safe_offboard");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    double takeoff_height;
    double circle_radius;
    std::string flight_mode;
    double pos_valid_time;
    double waypoint_valid_time;
    std::vector<double> fly_fence{-1.0, -1.0, 0.0, 1.0, 1.0, 1.0};

    nh.param<double>("takeoff_height", takeoff_height, 1.23);
    nh.param<double>("circle_radius", circle_radius, 1.23);
    nh.param<std::string>("flight_mode", flight_mode, std::string("stay"));
    nh.param<double>("pos_valid_time", pos_valid_time, 1.0);
    nh.param<double>("waypoint_valid_time", waypoint_valid_time, 1.0);
    nh.getParam("fly_fence",fly_fence);


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

