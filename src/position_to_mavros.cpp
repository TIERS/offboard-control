#include "position_to_mavros/position_to_mavros.h"

position_to_mavros::position_to_mavros(ros::NodeHandle& nh){
    ROS_INFO_STREAM(">>>>>> position_to_mavros initialization process <<<<<<");

    nh_ = nh;
    
    ros::NodeHandle nhh("~");
    nhh.param<bool>("only_vision", vision_only_, true);
    nhh.param<std::string>("pose_pub_topic", pose_pub_topic_, "/uav/mavros/vision_pose/pose");
    nhh.param<std::string>("uwb_sub_topic", uwb_sub_topic_, "/dwm1001/tag/dronie/position");
    nhh.param<std::string>("vision_sub_topic", vision_sub_topic_, "/uav/t265/odom/sample");
    nhh.param<std::string>("lidar_sub_topic", lidar_sub_topic_, "/uav/tfmini_ros_node/range");

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_pub_topic_, 5);
    if(!vision_only_){
        uwb_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(uwb_sub_topic_, 1, &position_to_mavros::uwb_callback, this);
        // current lidar is not used
        // lidar_sub_ = nh_.subscribe<sensor_msgs::Range>(lidar_sub_topic_, 1, &position_to_mavros::lidar_callback, this);
    }
    vision_sub_ = nh_.subscribe<nav_msgs::Odometry>(vision_sub_topic_, 1, &position_to_mavros::vision_callback, this); 
    
    lidar_z_ = 0.0;

    landing_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

}

position_to_mavros::~position_to_mavros(){}


void position_to_mavros::uwb_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO_STREAM(">>>>>> In UWB Callback <<<<<<");
    uwb_pos_.header.stamp = ros::Time::now();
    uwb_pos_.pose.position.x = msg->pose.position.x;
    uwb_pos_.pose.position.y = msg->pose.position.y;
    uwb_pos_.pose.position.z = msg->pose.position.z;
    uwb_pos_.pose.orientation.x = msg->pose.orientation.x;
    uwb_pos_.pose.orientation.y = msg->pose.orientation.y;
    uwb_pos_.pose.orientation.z = msg->pose.orientation.z;
    uwb_pos_.pose.orientation.w = msg->pose.orientation.w;
    vision_last_pos_ = vision_pos_;    
}

void position_to_mavros::lidar_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_INFO_STREAM(">>>>>> In Lidar Callback <<<<<<");
    lidar_z_ = msg->range;
}

void position_to_mavros::vision_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO_STREAM(">>>>>> In Vision Callback <<<<<<");
    vision_pos_.header.stamp = ros::Time::now();
    vision_pos_.header.frame_id = "/camera_frame";
    vision_pos_.pose.position.x = msg->pose.pose.position.x;
    vision_pos_.pose.position.y = msg->pose.pose.position.y;
    vision_pos_.pose.position.z = msg->pose.pose.position.z;
    vision_pos_.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pos_.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pos_.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pos_.pose.orientation.w = msg->pose.pose.orientation.w;
}


void position_to_mavros::run(const ros::TimerEvent& event){

    // -0.13 0.0 -0.12 
    //ROS_INFO_STREAM(">>>>>> " << "Before Vision Pose: " << vision_pos_.pose.position.x << " ," <<vision_pos_.pose.position.y << " ," <<vision_pos_.pose.position.z << " <<<<<<");
    if((ros::Time::now() - vision_pos_.header.stamp) > ros::Duration(0.2))
    { // if the vision pos is too old, do emergency landing.
        mavros_msgs::CommandTOL srv_land;
        srv_land.request.altitude = 0;
        srv_land.request.latitude = 0;
        srv_land.request.longitude = 0;
        srv_land.request.min_pitch = 0;
        srv_land.request.yaw = 0;
        if(landing_client_.call(srv_land))
        {
            ROS_INFO_STREAM("Vision Pose Stopped, Land.");
        }else
        {
            ROS_ERROR_STREAM("Vision Pose Stopped, Failed to Call Land Service!");
        }
    }
    if(vision_only_){
        geometry_msgs::PoseStamped transformed_pos;
        geometry_msgs::TransformStamped T;
        const geometry_msgs::PoseStamped& older_pos = vision_pos_;
        T.header.stamp = ros::Time::now();
        T.header.frame_id = "/tranform_frame";
        T.transform.translation.x = -0.13*cos(2*acos(vision_pos_.pose.orientation.w));
        T.transform.translation.y = -0.13*sin(2*acos(vision_pos_.pose.orientation.w));
        T.transform.translation.z = 0.0;

        // T.transform.setRotation( tf2::Quaternion(0, 0, 0, 1) );
        //  tf2::Quaternion q;
        // q.setRPY(0, 0, 0);
        T.transform.rotation.x = 0.0;//-vision_pos_.pose.orientation.x;//0.0;
        T.transform.rotation.y = 0.0;//-vision_pos_.pose.orientation.y;//0.0;
        T.transform.rotation.z = 0.0;//-vision_pos_.pose.orientation.z;//0.0;
        T.transform.rotation.w = 1.0;//-vision_pos_.pose.orientation.w;//1.0;

        tf2::doTransform(older_pos, transformed_pos, T);

        local_pos_.header.stamp = ros::Time::now();
        local_pos_.header.frame_id = "/transfromed_local_frame";
        local_pos_.pose.position.x = transformed_pos.pose.position.x;
        local_pos_.pose.position.y = transformed_pos.pose.position.y;
        local_pos_.pose.position.z = transformed_pos.pose.position.z;
        local_pos_.pose.orientation.x = transformed_pos.pose.orientation.x;
        local_pos_.pose.orientation.y = transformed_pos.pose.orientation.y;
        local_pos_.pose.orientation.z = transformed_pos.pose.orientation.z;
        local_pos_.pose.orientation.w = transformed_pos.pose.orientation.w;
    }
    else{
        local_pos_.header.stamp = ros::Time::now();
        local_pos_.header.frame_id = "/transfromed_local_frame";
        local_pos_.pose.position.x = uwb_pos_.pose.position.x + vision_pos_.pose.position.x -  vision_last_pos_.pose.position.x;
        local_pos_.pose.position.y = uwb_pos_.pose.position.y + vision_pos_.pose.position.y -  vision_last_pos_.pose.position.y;
        local_pos_.pose.position.z = uwb_pos_.pose.position.z + vision_pos_.pose.position.z -  vision_last_pos_.pose.position.z;
        local_pos_.pose.orientation.x =  vision_pos_.pose.orientation.x;
        local_pos_.pose.orientation.y =  vision_pos_.pose.orientation.y;
        local_pos_.pose.orientation.z =  vision_pos_.pose.orientation.z;
        local_pos_.pose.orientation.w =  vision_pos_.pose.orientation.w;
    }
    local_pos_pub_.publish(local_pos_);    
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "position_to_mavros");

    ros::NodeHandle n;
    // ros::NodeHandle nh("~");
    position_to_mavros pm = position_to_mavros(n);

    ros::Timer timer = n.createTimer(ros::Duration(0.02), &position_to_mavros::run, &pm);
    
    ros::spin();
    return 0;
}