#include "safe_offboard/safe_offboard.h"
#include <math.h>


safe_offboard::safe_offboard(ros::NodeHandle& nh)
{
    nh_ = nh;
    ros::NodeHandle nhh("~");
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    emergency_landing_ = false;
    
    current_pos_.pose.position.x = -1;
    current_pos_.pose.position.y = -1;
    current_pos_.pose.position.z = -1;

    flight_mode_ = "stay";
    takeoff_height_ = 1.0;
    circle_radius_ = 1.0;
    pos_valid_time_ = 1.0;
    waypoint_valid_time_ = 1.0;
    emergency_landing_ = false;
    taken_off_ = false;

    offboard_state_ = "disarmed"; // ["disarmed", "armed", "taking_off", "flying", "landing", "emergency", "going_home"]

    std::vector<double> flying_fence {-2.0,-2.0, 0.0, 2.0, 2.0, 2.0};
    nhh.param<double>("takeoff_height", takeoff_height_, 1.23);
    nhh.param<double>("circle_radius", circle_radius_, 1.23);
    nhh.param<std::string>("flight_mode", flight_mode_, std::string("stay"));
    nhh.param<double>("pos_valid_time", pos_valid_time_, 1.0);
    nhh.param<double>("waypoint_valid_time", waypoint_valid_time_, 1.0);
    nhh.getParam("fly_fence",flying_fence);
    fly_fence_.lower_point = point{flying_fence[0], flying_fence[1], flying_fence[2]};
    fly_fence_.upper_point = point{flying_fence[3], flying_fence[4], flying_fence[5]};
    ROS_INFO_STREAM("fly fence lower point: " << fly_fence_.lower_point.x << " , " << fly_fence_.lower_point.y << " , " << fly_fence_.lower_point.z);
    ROS_INFO_STREAM("fly fence upper point: " << fly_fence_.upper_point.x << " , " << fly_fence_.upper_point.y << " , " << fly_fence_.upper_point.z);

    std::string mavros_state_sub_topic;
    std::string mavros_position_sub_topic;
    std::string external_waypoint_sub_topic; 
    std::string waypoint_pub_topic;
    std::string offboard_state_pub_topic;
    std::string flight_mode_pub_topic;
    std::string arming_client_topic;
    std::string landing_client_topic; 
    std::string mavros_set_mode_client_topic;
    std::string emergency_land_server_topic;
    std::string flight_mode_sub_topic;
    std::string flight_mode_srv_topic;
    std::string offboard_state_srv_topic;

    nhh.param<std::string>("mavros_state_sub_topic", mavros_state_sub_topic, "mavros/state");
    nhh.param<std::string>("mavros_position_sub_topic", mavros_position_sub_topic, "mavros/vision_pose/pose");
    nhh.param<std::string>("external_waypoint_sub_topic", external_waypoint_sub_topic, "offboard/command_waypoint");
    nhh.param<std::string>("waypoint_pub_topic", waypoint_pub_topic, "mavros/setpoint_position/local");

    nhh.param<std::string>("offboard_state_pub_topic", offboard_state_pub_topic, "safe_offboard_state");
    nhh.param<std::string>("flight_mode_pub_topic", flight_mode_pub_topic, "flight_mode");
    nhh.param<std::string>("arming_client_topic", arming_client_topic, "mavros/cmd/arming");
    nhh.param<std::string>("landing_client_topic", landing_client_topic, "mavros/cmd/land");

    nhh.param<std::string>("mavros_set_mode_client_topic", mavros_set_mode_client_topic, "mavros/set_mode");
    nhh.param<std::string>("emergency_land_server_topic", emergency_land_server_topic, "offboard/emergency_land");
    nhh.param<std::string>("flight_mode_sub_topic", flight_mode_sub_topic, "offboard/mode");
    nhh.param<std::string>("flight_mode_srv_topic", flight_mode_srv_topic, "offboard/cmd/mode");
    nhh.param<std::string>("offboard_state_srv_topic", offboard_state_srv_topic, "offboard/state");



    state_sub_ = nh_.subscribe<mavros_msgs::State>(mavros_state_sub_topic, 10, &safe_offboard::state_cb, this);
    position_cb_ = nh_.subscribe<geometry_msgs::PoseStamped>(mavros_position_sub_topic, 10, &safe_offboard::update_current_pos, this);
    external_waypoint_cb_ = nh_.subscribe<geometry_msgs::PoseStamped>(external_waypoint_sub_topic, 10, &safe_offboard::update_external_waypoint, this);

    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(waypoint_pub_topic, 10);
    offboard_state_pub_ = nh_.advertise<std_msgs::String>(offboard_state_pub_topic, 10);
    flight_mode_pub_ = nh_.advertise<std_msgs::String>(flight_mode_pub_topic, 10);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);
    landing_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(landing_client_topic);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(mavros_set_mode_client_topic);
    emergency_land_server_ = nh_.advertiseService(emergency_land_server_topic, &safe_offboard::emergency_srv_cb, this);
    flight_mode_sub_ = nh_.subscribe<std_msgs::String>(flight_mode_sub_topic, 10, &safe_offboard::mode_cb, this);

    flight_mode_srv_ = nh_.advertiseService(flight_mode_srv_topic, &safe_offboard::flight_mode_srv_cb, this);
    offboard_state_srv_ = nh_.advertiseService(offboard_state_srv_topic, &safe_offboard::offboard_state_srv_cb, this);
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
    else if (msg->data == "stay")
    {
        flight_mode_ = "stay";
    }
    else if (msg->data == "hover")
    {
        flight_mode_ = "hover";
    }
    else if (msg->data == "circle")
    {
        flight_mode_ = "circle";
    }
    else if (msg->data == "external_control")
    {
        flight_mode_ = "external_control";
    }
    else{
        // keep the last mode
    }
}


bool safe_offboard::flight_mode_srv_cb(offboard_control::flight_mode::Request &request, offboard_control::flight_mode::Response &response)
{
    if (request.mode == "land")
    {
        flight_mode_ = "land";
    }
    else if (request.mode == "stay")
    {
        flight_mode_ = "stay";
    }
    else if (request.mode == "hover")
    {
        flight_mode_ = "hover";
    }
    else if (request.mode == "circle")
    {
        flight_mode_ = "circle";
    }
    else if (request.mode == "external_control")
    {   
        if(offboard_state_ == "flying")
            flight_mode_ = "external_control";
        else
        {
            response.feedback = "UAV Is Not in 'flying' state, Flight Mode Set Failed.";
            return false;
        }
    }
    else{
        response.feedback = "Flight Mode Set Succeeded.";
        return false;
    }
    
    response.feedback = "Flight Mode Set Succeeded.";
    return true;
}


bool safe_offboard::offboard_state_srv_cb(offboard_control::offboard_state::Request &request, offboard_control::offboard_state::Response &response)
{
    if (request.state == "disarmed")
    {
        offboard_state_ = "disarmed";
    }
    else if (request.state == "armed")
    {
        offboard_state_ = "armed";
    }
    else if (request.state == "taking_off")
    {
        offboard_state_ = "taking_off";
    }
    else if (request.state == "flying")
    {
        offboard_state_ = "flying";
    }
    else if (request.state == "landing")
    {
        offboard_state_ = "landing";
    }
    else if(request.state == "emergency")
    {
        offboard_state_ = "emergency";
    }
    else if(request.state == "going_home")
    {
        offboard_state_ = "going_home";
    }
    else{
        response.feedback = "Offboard State Set Failed";
        return false;
    }
    
    response.feedback = "Offboard State Set Succeeded";
    return true;
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


void safe_offboard::pub_offboard_state(const ros::TimerEvent& event)
{   
    std_msgs::String msg;
    msg.data = offboard_state_;
    offboard_state_pub_.publish(msg);
    std_msgs::String m;
    m.data = flight_mode_;
    flight_mode_pub_.publish(m);
}

void safe_offboard::run()
{
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

    safe_offboard so = safe_offboard(n);
    ros::Timer timer = n.createTimer(ros::Duration(0.05), &safe_offboard::check_poses, &so);
    ros::Timer time_0 = n.createTimer(ros::Duration(0.5), &safe_offboard::pub_offboard_state, &so);
    so.run();
    
    ros::spin();
    return 0;
}

