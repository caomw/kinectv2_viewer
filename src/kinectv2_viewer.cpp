/**
@author: Robin Singh
@email : robnsngh@umich.edu

Simle Point Cloud Viewer for Kinect ONE.  

Namespace will be expanded into more functionaliy (and I am bad with spellings)

**/


//ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

//PCL headers
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


namespace KinectV2
{

class Viewer
{
private:

  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_; 
  pcl::visualization::CloudViewer viewer_;

public:

  Viewer(bool flag) : 
    nh_("~"),
    viewer_("Simple Cloud Viewer")
  {
    point_cloud_sub_ = nh_.subscribe("/kinect2_head/depth_highres/points", 1, &Viewer::cloudCallback, this);
    ROS_INFO("Looking for Point Cloud ");
  }
  void cloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    ROS_INFO_STREAM("PC Received");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    viewer_.showCloud(cloud);
  }

};

};

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Simple KinectV2 Point Cloud Viewer");

  ros::init(argc, argv, "KinectV2");

  KinectV2::Viewer detector(true);

  ros::spin();
  return 0;
}

