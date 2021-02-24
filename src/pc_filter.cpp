#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <memory>

float x_clip_min_;
float x_clip_max_;
float y_clip_min_;
float y_clip_max_;
float z_clip_min_;
float z_clip_max_;

int r_;
int g_;
int b_;

std::string filtered_frame_id;
std::string observed_frame_id;
std::string input_pc_topic;
std::string output_pc_topic;
std::string ugv_center_xy_topic;

ros::Publisher filtered_pc_pub;
ros::Publisher ugv_center_xy_pub;

void filterCallback(const sensor_msgs::PointCloud2ConstPtr& sensor_message_pc)
{
  ROS_DEBUG_STREAM(">>>>>>>> Filtering PointCloud Callback <<<<<<<<");
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*sensor_message_pc, original_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_green_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PassThrough<pcl::PointXYZRGB > pass;

  ROS_DEBUG_STREAM("filter cloudsize before x: " << original_pc->size());

  pass.setInputCloud(original_pc);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_clip_min_, x_clip_max_);
  pass.filter(*cloud_filtered_x);

  ROS_DEBUG_STREAM("filter cloudsize before y: " << cloud_filtered_x->size());

  pass.setInputCloud(cloud_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_clip_min_, y_clip_max_);
  pass.filter(*cloud_filtered_xy);

  std::cout << "filter cloudsize before z: " << cloud_filtered_xy->size() << std::endl;

  pass.setInputCloud(cloud_filtered_xy);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_clip_min_, z_clip_max_);
  pass.filter(*cloud_filtered_xyz);
  
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  for (int i = 0; i < cloud_filtered_xyz->points.size(); i++)
  {
    int r = cloud_filtered_xyz->points[i].r;
    int g = cloud_filtered_xyz->points[i].g;
    int b = cloud_filtered_xyz->points[i].b;
    if (r < r_ && g > g_ && b < b_) {
      inliers->indices.push_back(i);
    }
    
  }
  extract.setIndices(inliers);
  extract.setInputCloud(cloud_filtered_xyz);
  extract.setNegative(false);
  extract.filter(*cloud_green_xyz);

  double sum_x = 0.0;
  double sum_y = 0.0;
  for(auto iter = cloud_green_xyz->points.begin(); iter != cloud_green_xyz->points.end(); ++iter)
  {
    sum_x += iter->x;
    sum_y += iter->y;
  }

  double avg_x = sum_x / cloud_green_xyz->points.size();
  double avg_y = sum_y / cloud_green_xyz->points.size();

  if(cloud_green_xyz->points.size() == 0){
    ROS_INFO_STREAM("No Green Tag In the UGV Center");
    return ;
  }

  ROS_INFO_STREAM("Points Size: " << cloud_green_xyz->points.size());
  ROS_INFO_STREAM("Average x and y: " << avg_x << " , " << avg_y);

  /** This is a new way to filter color so keep it 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
        green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, 90));
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (green_condition);

    // Build the filter
    color_filter.setInputCloud(cloud_filtered_xyz);
    color_filter.setCondition (color_cond);
    color_filter.setUserFilterValue(1.0);
    color_filter.filter(*cloud_filtered);
  */
    
  // pcl::PCLPointCloud2Ptr cloud_transformed_back_pc2(new pcl::PCLPointCloud2());
  // pcl::toPCLPointCloud2(*cloud_transformed_back, *cloud_transformed_back_pc2);

  /** keep this for downsampling use
    pcl::PCLPointCloud2 cloud_filtered_downsampled;
    if(downsample_) {
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud_transformed_back_pc2);
      sor.setLeafSize (downsample_, downsample_, downsample_);
      sor.filter (cloud_filtered_downsampled);
    } else {
      cloud_filtered_downsampled = *cloud_transformed_back_pc2;
    }
  */
  geometry_msgs::PoseStamped center_msg;
  center_msg.header.stamp = ros::Time::now();
  center_msg.pose.position.x = avg_x;
  center_msg.pose.position.y = avg_y;
  ugv_center_xy_pub.publish(center_msg);

  sensor_msgs::PointCloud2 cloud_back_msg;
  cloud_back_msg.header.frame_id = cloud_green_xyz->header.frame_id;
  pcl::toROSMsg <pcl::PointXYZRGB> (*cloud_green_xyz, cloud_back_msg);
  filtered_pc_pub.publish(cloud_back_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_filter_node");
  ros::NodeHandle n_;
  // n = &n_;

  n_.getParam("xpassthrough/filter_limit_min", x_clip_min_);
  n_.getParam("xpassthrough/filter_limit_max", x_clip_max_);
  n_.getParam("ypassthrough/filter_limit_min", y_clip_min_);
  n_.getParam("ypassthrough/filter_limit_max", y_clip_max_);
  n_.getParam("zpassthrough/filter_limit_min", z_clip_min_);
  n_.getParam("zpassthrough/filter_limit_max", z_clip_max_);
  n_.getParam("target_color/r", r_);
  n_.getParam("target_color/g", g_);
  n_.getParam("target_color/b", b_);

  n_.getParam("observed_frame_id", observed_frame_id);
  n_.getParam("filtered_frame_id", filtered_frame_id);
  n_.getParam("input_pc_topic", input_pc_topic);
  n_.getParam("output_pc_topic", output_pc_topic);
  n_.getParam("ugv_center_xy_topic", ugv_center_xy_topic);

  ROS_INFO_STREAM("Listening on this input pc topic: " << input_pc_topic);
  ROS_INFO_STREAM("Listening on this output pc topic: " << output_pc_topic);
  ROS_INFO_STREAM("Listening on this observed_frame_id: " << observed_frame_id);
  ROS_INFO_STREAM("Listening on this filtered_frame_id: " << filtered_frame_id);
  ROS_INFO_STREAM("Listening on this ugv_center_xy_topic: " << ugv_center_xy_topic);
  ROS_INFO_STREAM("Listening on this targeted color rgb: " << r_ << " , " << g_ << " , " << b_ << input_pc_topic);
  ROS_INFO_STREAM("Dimensions for filtered scene are: (" << x_clip_min_ << ", " << x_clip_max_ << ") (" << y_clip_min_ << ", " << y_clip_max_ << ") (" << z_clip_min_ << ", " << z_clip_max_ << ")");

  ros::Time now = ros::Time::now();

  ros::Subscriber original_pc_sub = n_.subscribe(input_pc_topic, 1, filterCallback);

  filtered_pc_pub = n_.advertise<sensor_msgs::PointCloud2>(output_pc_topic, 1);

  ugv_center_xy_pub = n_.advertise<geometry_msgs::PoseStamped>(ugv_center_xy_topic, 1);

  ros::spin();
  return 0;
}

