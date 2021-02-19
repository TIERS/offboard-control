/*
 * @Author: Tiers - Xianjia 
 * @Date:   2021-02-09 21:56:02
 * @Last Modified by:   Your name
 * @Last Modified time: 2021-02-11 12:59:13
 */
#include "position_to_mavros/position_to_mavros.h"

position_to_mavros::position_to_mavros(ros::NodeHandle& nh, bool vision_only){
    ROS_INFO_STREAM(">>>>>> position_to_mavros initialization process <<<<<<");

    nh_ = nh;

    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/uav/mavros/vision_pose/pose", 5);
    if(!vision_only){
        uwb_sub = nh_.subscribe<geometry_msgs::Pose>("/dwm1001/tag/dronie/position", 1, &position_to_mavros::uwb_callback, this);

        lidar_sub = nh_.subscribe<sensor_msgs::Range>("/uav/tfmini_ros_node/range", 1, &position_to_mavros::lidar_callback, this);
    }
    vision_sub = nh_.subscribe<nav_msgs::Odometry>("/uav/camera/odom/sample", 1, &position_to_mavros::vision_callback, this); 
    

    lidar_z = 0.0;

}

position_to_mavros::~position_to_mavros(){}


void position_to_mavros::uwb_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_STREAM(">>>>>> In UWB Callback <<<<<<");
    uwb_pos.header.stamp = ros::Time::now();
    uwb_pos.pose.position.x = msg->position.x;
    uwb_pos.pose.position.y = msg->position.y;
    uwb_pos.pose.position.z = lidar_z;

    local_pos_pub.publish(uwb_pos);
    
}

void position_to_mavros::lidar_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO_STREAM(">>>>>> In Lidar Callback <<<<<<");
    lidar_z = msg->range;
}

void position_to_mavros::vision_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO_STREAM(">>>>>> In Vision Callback <<<<<<");
    vision_pos.header.stamp = ros::Time::now();
    vision_pos.header.frame_id = "/camera_frame";
    vision_pos.pose.position.x = msg->pose.pose.position.x;
    vision_pos.pose.position.y = msg->pose.pose.position.y;
    vision_pos.pose.position.z = msg->pose.pose.position.z;
    vision_pos.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pos.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pos.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pos.pose.orientation.w = msg->pose.pose.orientation.w;
}


void position_to_mavros::run(const ros::TimerEvent& event){

 // -0.13 0.0 -0.12 
    // static tf2_ros::TransformBroadcaster br;
    //ROS_INFO_STREAM(">>>>>> " << "Before Vision Pose: " << vision_pos.pose.position.x << " ," <<vision_pos.pose.position.y << " ," <<vision_pos.pose.position.z << " <<<<<<");
    geometry_msgs::PoseStamped transformed_pos;
    geometry_msgs::TransformStamped T;
    const geometry_msgs::PoseStamped& older_pos = vision_pos;

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "/tranform_frame";
    T.transform.translation.x = -0.13*cos(2*acos(vision_pos.pose.orientation.w));
    T.transform.translation.y = -0.13*sin(2*acos(vision_pos.pose.orientation.w));
    T.transform.translation.z = 0.0;

    // T.transform.setRotation( tf2::Quaternion(0, 0, 0, 1) );
    //  tf2::Quaternion q;
    // // q.setRPY(0, 0, 0);
    T.transform.rotation.x = 0.0;//-vision_pos.pose.orientation.x;//0.0;
    T.transform.rotation.y = 0.0;//-vision_pos.pose.orientation.y;//0.0;
    T.transform.rotation.z = 0.0;//-vision_pos.pose.orientation.z;//0.0;
    T.transform.rotation.w = 1.0;//-vision_pos.pose.orientation.w;//1.0;

    tf2::doTransform(older_pos, transformed_pos, T);

    local_pos.header.stamp = ros::Time::now();
    local_pos.header.frame_id = "/transfromed_local_frame";
    local_pos.pose.position.x = transformed_pos.pose.position.x;
    local_pos.pose.position.y = transformed_pos.pose.position.y;
    local_pos.pose.position.z = transformed_pos.pose.position.z;
    local_pos.pose.orientation.x = transformed_pos.pose.orientation.x;
    local_pos.pose.orientation.y = transformed_pos.pose.orientation.y;
    local_pos.pose.orientation.z = transformed_pos.pose.orientation.z;
    local_pos.pose.orientation.w = transformed_pos.pose.orientation.w;


    // T.header.stamp = ros::Time::now();
    // T.header.frame_id = "/tranform_frame";
    // T.transform.translation.x = -0.13;
    // T.transform.translation.y = 0.0;
    // T.transform.translation.z = 0.0;
    // // tf2::Quaternion q;
    // // q.setRPY(0, 0, 0);
    // T.transform.rotation.x = 0.0;
    // T.transform.rotation.y = 0.0;
    // T.transform.rotation.z = 0.0;
    // // T.transform.rotation.w = 0.0;

    // tf2::doTransform(older_pos, transformed_pos, T);
    local_pos.header.stamp = ros::Time::now();
    local_pos.header.frame_id = "/transfromed_local_frame";
    local_pos.pose.position.x = vision_pos.pose.position.x;
    local_pos.pose.position.y = vision_pos.pose.position.y;
    local_pos.pose.position.z = vision_pos.pose.position.z;
    local_pos.pose.orientation.x = vision_pos.pose.orientation.x;
    local_pos.pose.orientation.y = vision_pos.pose.orientation.y;    
    local_pos.pose.orientation.z = vision_pos.pose.orientation.z;
    local_pos.pose.orientation.w = vision_pos.pose.orientation.w;
    // ROS_INFO_STREAM(">>>>>> " << "After Vision Pose: " << transformed_pos.pose.position.x << " ," << transformed_pos.pose.position.y << " ," << transformed_pos.pose.position.z << " <<<<<<");
    // geometry_msgs::PoseStamped transformed_pos;

    // br.sendTransform(transformStamped);

    local_pos_pub.publish(local_pos);    
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "position_to_mavros");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    position_to_mavros pm = position_to_mavros(n);

    ros::Timer timer = n.createTimer(ros::Duration(0.02), &position_to_mavros::run, &pm);
    
    ros::spin();
    return 0;
}