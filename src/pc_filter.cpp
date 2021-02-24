#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
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
float dist_threshold_;
int min_pts_;
int max_pts_;

int r_;
int g_;
int b_;

std::string filtered_frame_id;
std::string observed_frame_id;
std::string input_pc_topic;
std::string output_pc_topic;
std::string ugv_center_xy_topic;
std::string uav_big_tag_xy_topic;
std::string uav_small_tag_xy_topic;

ros::Publisher filtered_pc_pub;
ros::Publisher ugv_center_xy_pub;
ros::Publisher uav_big_tag_xy_pub;
ros::Publisher uav_small_tag_xy_pub;



bool campare_condition (const pcl::PointIndices p_i, const pcl::PointIndices p_j) { return p_i.indices.size() > p_j.indices.size(); }


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
  double avg_small_x = 99999.000;
  double avg_small_y = 99999.000;
  double avg_big_x = 99999.000;
  double avg_big_y = 99999.000;

  if(cloud_green_xyz->points.size() >0){
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_search(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_search->setInputCloud(cloud_green_xyz);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(dist_threshold_);  
    ec.setMinClusterSize(min_pts_);
    ec.setMaxClusterSize(max_pts_);
    ec.setSearchMethod(tree_search);
    ec.setInputCloud(cloud_green_xyz);
    ec.extract(cluster_indices);

    ROS_INFO_STREAM("Cluster Size: " << cluster_indices.size());
    
    std::vector<int> big_cluster, small_cluster;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>());
  if(cluster_indices.size() >=1 )
  {

    std::sort(cluster_indices.begin(), cluster_indices.end(), campare_condition);
    big_cluster = cluster_indices[0].indices;
    double sum_big_x = 0.0;
    double sum_big_y = 0.0;
    for (std::vector<int>::const_iterator pit = big_cluster.begin(); pit != big_cluster.end(); ++pit)
    {
      // cloud_0->points.push_back(cloud_green_xyz->points[*pit]); 
      sum_big_x += cloud_green_xyz->points[*pit].x;
      sum_big_y += cloud_green_xyz->points[*pit].y;
    }
    avg_big_x = sum_big_x / big_cluster.size();
    avg_big_y = sum_big_y / big_cluster.size();
    ROS_INFO_STREAM("Big Tag: Points Size: " << big_cluster.size());

    geometry_msgs::PoseStamped big_tag_msg;
    big_tag_msg.header.stamp = ros::Time::now();
    big_tag_msg.pose.position.x = avg_big_x;
    big_tag_msg.pose.position.y = avg_big_y;
    uav_big_tag_xy_pub.publish(big_tag_msg);

    if(cluster_indices.size() > 1)
    {
      small_cluster = cluster_indices[1].indices;
      double sum_small_x = 0.0;
      double sum_small_y = 0.0;
      for (std::vector<int>::const_iterator pit = small_cluster.begin(); pit != small_cluster.end(); ++pit)
      {
        // cloud_1->points.push_back(cloud_green_xyz->points[*pit]); 
        sum_small_x += cloud_green_xyz->points[*pit].x;
        sum_small_y += cloud_green_xyz->points[*pit].y;
      }

      avg_small_x = sum_small_x / small_cluster.size();
      avg_small_y = sum_small_y / small_cluster.size();

      ROS_INFO_STREAM("Small Tag: Points Size: " << small_cluster.size());

      geometry_msgs::PoseStamped small_tag_msg;
      small_tag_msg.header.stamp = ros::Time::now();
      small_tag_msg.pose.position.x = avg_small_x;
      small_tag_msg.pose.position.y = avg_small_y;
      uav_small_tag_xy_pub.publish(small_tag_msg);
    }
    }
  }
  else{
    ROS_INFO_STREAM("No Green Tag In the UGV Center");
  }  

  ROS_INFO_STREAM("Big Tag: Average x and y: " << avg_big_x << " , " << avg_big_y);
  ROS_INFO_STREAM("Small Tag: Average x and y: " << avg_small_x << " , " << avg_small_y);


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
  n_.getParam("cluster/dist_threshold", dist_threshold_);
  n_.getParam("cluster/min_pts", min_pts_);
  n_.getParam("cluster/max_pts", max_pts_);


  n_.getParam("observed_frame_id", observed_frame_id);
  n_.getParam("filtered_frame_id", filtered_frame_id);
  n_.getParam("input_pc_topic", input_pc_topic);
  n_.getParam("output_pc_topic", output_pc_topic);
  n_.getParam("ugv_center_xy_topic", ugv_center_xy_topic);
  n_.getParam("uav_big_tag_xy_topic", uav_big_tag_xy_topic);
  n_.getParam("uav_small_tag_xy_topic", uav_small_tag_xy_topic);

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

  uav_big_tag_xy_pub = n_.advertise<geometry_msgs::PoseStamped>(uav_big_tag_xy_topic, 1);

  uav_small_tag_xy_pub = n_.advertise<geometry_msgs::PoseStamped>(uav_small_tag_xy_topic, 1);

  

  ros::spin();
  return 0;
}

  // geometry_msgs::PoseStamped center_msg;
  // center_msg.header.stamp = ros::Time::now();
  // center_msg.pose.position.x = avg_x;
  // center_msg.pose.position.y = avg_y;
  // ugv_center_xy_pub.publish(center_msg);


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