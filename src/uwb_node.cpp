#include <uwb.h>
#include <vector> 
#include <stdio.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfmini_ros_node");
  ros::NodeHandle nh("~");
  std::string id = "uwb_range";
  std::string port_name;
  int baud_rate;
  tiersuwb::UWB *uwb_obj;

  nh.param("serial_port", portName, std::string("/dev/uwb"));
  nh.param("baud_rate", baud_rate, 115200);

  ROS_INFO_STREAM("Trying to connect to TIERS 4by4 UWB ...");
  uwb_obj = new tiersuwb::UWB(port_name, baud_rate);
  ROS_INFO_STREAM("Connected successfully!!");
  
  // Publishers
  char transceiver_names[4] = ['A', 'B', 'C', 'D'];
  std::vector<vector<ros::Publisher>> publishers;
  for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
          if (i != j) {
              char name_buffer [50];
              std:sprintf(name_buffer, "/uwb/distance/from/%s/to/%s", transceiver_names[i], transceiver_names[j])
              publishers[i][j] = nh.advertise<sensor_msgs::Range>(name_buffer, 10, true);
          }
      }
  }

  sensor_msgs::Range uwb_range;

  uwb_range.radiation_type = 2;         // UWB ?
  uwb_range.field_of_view = 3.14;
  uwb_range.min_range = 0.3;
  uwb_range.max_range = 30;
  uwb_range.header.frame_id = id;
  float dist = 0;
  ROS_INFO_STREAM("Start processing ...");

  while(ros::master::check() && ros::ok())
  {
    ros::spinOnce();
    dist = tfmini_obj->getDist();
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist;
      TFmini_status.data = "ok";
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);
    }
    else if(dist == -1.0)
    {
      ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      TFmini_status.data = "error";
      break;
    }
    else if(dist == 0.0)
    {
      ROS_ERROR_STREAM("Data validation error!");
      TFmini_status.data = "error";
    }
    else {
      TFmini_status.data = "out";
    }
    pub_status.publish(TFmini_status);
  }

  tfmini_obj->closePort();
}