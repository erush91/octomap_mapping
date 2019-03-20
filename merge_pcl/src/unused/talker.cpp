///////////////////////////////////////////////////////////////////////////////
// Function Name : Motion Control
// Purpose : Class to store and run a physics based motion controller
///////////////////////////////////////////////////////////////////////////////
// Changelog :
// Date           % Name       %   Reason
// 03 / 04 / 2019 % Gene Rush  %   Created - Added general structure
///////////////////////////////////////////////////////////////////////////////i

// ***ONLINE REFERNCES***
// INCLUDES FOR CONVERTING POINT CLOUDS
// https://answers.ros.org/question/204244/build-errorconverting-pclpclpointcloud2-to-sensor_msgpointcloud2/
// CONCATENATE POINT CLOUDS
// https://answers.ros.org/question/243588/how-to-use-concatenatepointcloud/
// CMAKELISTS.TXT FOR POINT CLOUDS
// https://github.com/ros-perception/perception_pcl/blob/hydro-devel/pcl_ros/CMakeLists.txt
// https://answers.ros.org/question/81306/catkin-cant-find-pclconfigcmake/
// https://answers.ros.org/question/81306/catkin-cant-find-pclconfigcmake/
// PUBLISH/SUBSCRIBE
// https://github.com/team-vigir/vigir_lidar_proc/blob/master/vigir_lidar_intensity_modifier/src/laser_intensity_modifier_node.cpp#L52
// VOXEL GRID FILTER
// https://stackoverflow.com/questions/43245726/pcl-downsample-with-pclvoxelgrid

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudFuser
{
public:

  PointCloudFuser()
  {
    ros::NodeHandle nh_;
    
    pclf_sub_5_ = nh_.subscribe("marble_nuc5/octomap_point_cloud_centers", 1000, &PointCloudFuser::pclfCallback, this);
    pclf_sub_6_ = nh_.subscribe("marble_nuc6/octomap_point_cloud_centers", 1000, &PointCloudFuser::pclfCallback, this);
    pclf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pclf", 1000);
  }
 
  ///////////////////////////////////////////////////////////////////////////////
  //      Function Name : pclfCallback
  //      Purpose : Callback Function, fuses point clouds and publishes
  //      In:             sensor_msgs::PointCLoud2 - sen_PCL 
  //      Out:    
  ///////////////////////////////////////////////////////////////////////////////
  void pclfCallback(const sensor_msgs::PointCloud2 sen_PCL)
  {
    // Merge point clouds
    sen_PCL_merge = fusePCL(sen_PCL);
    
    // Publish merged sensor_msgs::PointCloud2
    pclf_pub_.publish(sen_PCL_merge);
  }

  void pclf_cb(const sensor_msgs::PointCloud2 sen_PCL)
  {
    ROS_DEBUG("hi");
  }
  
  ///////////////////////////////////////////////////////////////////////////////
  //      Function Name : fusePCL
  //      Purpose : Fuses point clouds together
  //      In:             sensor_msgs::PointCLoud2 - sen_PCL 
  //      Out:    
  ///////////////////////////////////////////////////////////////////////////////
  sensor_msgs::PointCloud2 fusePCL(const sensor_msgs::PointCloud2 sen_PCL)
  {
    // Convert the input sensor_msgs::PointCloud2 to input pcl::PointCloud2
    pcl_conversions::toPCL(sen_PCL,pcl_PCL);

    // Debugging
    ROS_INFO("%i",pcl::getFieldsList(sen_PCL));

    // Concatenate the input pcl::PointCloud2 with the concatenated pcl::PointCloud2
    pcl::concatenatePointCloud(pcl_PCL_merge, pcl_PCL, pcl_PCL_merge);

    pcl::PCLPointCloud2::Ptr cloud_filt (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_unfilt (new pcl::PCLPointCloud2 ());
    *cloud_unfilt = pcl_PCL_merge;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_unfilt);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filt);    
    pcl_PCL_merge = *cloud_filt;
 
    // Convert the concatenated pcl::PointCloud to concatenated sensor_msgs::PointCloud2
    pcl_conversions::fromPCL(pcl_PCL_merge, sen_PCL_merge);

    // Debugging
    ROS_INFO("%i",pcl::getFieldsList(sen_PCL_merge));

    ROS_INFO("I heard boo");

    return sen_PCL_merge;
  }

protected:
  ros::Subscriber pclf_sub_5_;
  ros::Subscriber pclf_sub_6_; 
  ros::Publisher pclf_pub_;
  
  // Declare a concatenated sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 sen_PCL_merge;
  
  // Declare a pcl::PCLPointCloud2
  pcl::PCLPointCloud2 pcl_PCL;
  
  // Declare a concatenated pcl::PCLPointCloud2
  pcl::PCLPointCloud2 pcl_PCL_merge;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_point_cloud_centers_fusion_node");
  ros::NodeHandle nh_;
  ros::Publisher pclf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pclf", 1000);
  ros::Rate loop_rate(0.25);

  int count = 0;
  while(ros::ok())
  {
    ros::Subscriber pclf_sub_5_ = nh_.subscribe("marble_nuc5/octomap_point_cloud_centers", 1000, &PointCloudFuser::pclf_cb);
    ros::Subscriber pclf_sub_6_ = nh_.subscribe("marble_nuc6/octomap_point_cloud_centers", 1000, &PointCloudFuser::pclf_cb);
    &PointCloudFuser::pclfCallback;

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}

