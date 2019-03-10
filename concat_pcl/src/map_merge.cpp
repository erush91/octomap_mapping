///////////////////////////////////////////////////////////////////////////////
// Function Name : Motion Control
// Purpose : Class to store and run a physics based motion controller
///////////////////////////////////////////////////////////////////////////////
// Changelog :
// Date           % Name       %   Reason
// 03 / 04 / 2019 % Gene Rush  %   Created - Added general structure
///////////////////////////////////////////////////////////////////////////////

// ***ONLINE REFERNCES***
// INCLUDES FOR CONVERTING POINT CLOUDS
// https://answers.ros.org/question/204244/build-errorconverting-pclpclpointcloud2-to-sensor_msgpointcloud2/
// CONCATENATE POINT CLOUDS
// https://answers.ros.org/question/243588/how-to-use-concatenatepointcloud/
// CMAKELISTS.TXT FOR POINT CLOUDS
// https://answers.ros.org/question/81306/catkin-cant-find-pclconfigcmake/
// https://answers.ros.org/question/81306/catkin-cant-find-pclconfigcmake/
// PUBLISH/SUBSCRIBE
// https://github.com/team-vigir/vigir_lidar_proc/blob/master/vigir_lidar_intensity_modifier/src/laser_intensity_modifier_node.cpp#L52
// VOXEL GRID FILTER
// https:/`/stackoverflow.com/questions/43245726/pcl-downsample-with-pclvoxelgrid

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::cout;
using std::endl;

class PointCloudFuser
{
public:

  sensor_msgs::PointCloud2 sen_PCL_1;
  sensor_msgs::PointCloud2 sen_PCL_2;
  sensor_msgs::PointCloud2 sen_PCL_merge;

  ///////////////////////////////////////////////////////////////////////////////
  //      Function Name : pclCallback_1
  //      Purpose : Callback Function, gets sensor_msgs::PointCLoud2 - sen_PCL_1
  //      Out:    
  ///////////////////////////////////////////////////////////////////////////////
  void pclCallback_1(const sensor_msgs::PointCloud2 sen_PCL)
  {
      sen_PCL_1 = sen_PCL;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //      Function Name : pclCallback_2
  //      Purpose : Callback Function, gets sensor_msgs::PointCLoud2 - sen_PCL_2
  //      Out:    
  ///////////////////////////////////////////////////////////////////////////////
  void pclCallback_2(const sensor_msgs::PointCloud2 sen_PCL)
  {
      sen_PCL_2 = sen_PCL;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //      Function Name : fusePCL
  //      Purpose : Fuses point clouds together
  //      In:             sensor_msgs::PointCLoud2 - sen_PCL_1
  //      		  sensor_msgs::PointCLoud2 - sen_PCL_2
  //      Out:    
  ///////////////////////////////////////////////////////////////////////////////
  sensor_msgs::PointCloud2 fusePCL(sensor_msgs::PointCloud2 sen_PCL_1, sensor_msgs::PointCloud2 sen_PCL_2)
  {
    // Convert sensor_msgs::PointCloud2 to pcl::PCLPointCLoud2
    pcl_conversions::toPCL(sen_PCL_1,pcl_PCL_1);
    pcl_conversions::toPCL(sen_PCL_2,pcl_PCL_2);
      
    // Concatenate the input pcl::PointCloud2 with the concatenated pcl::PointCloud2
    pcl::concatenatePointCloud(pcl_PCL_1, pcl_PCL_2, pcl_PCL_merge);

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
    ROS_INFO("merged sen PCL width: %d",sen_PCL_merge.width);
 
    return sen_PCL_merge;
  }

protected:
  
  // Declare a pcl::PCLPointCloud2
  pcl::PCLPointCloud2 pcl_PCL_1;
  
  // Declare a pcl::PCLPointCloud2
  pcl::PCLPointCloud2 pcl_PCL_2;

  // Declare a concatenated pcl::PCLPointCloud2
  pcl::PCLPointCloud2 pcl_PCL_merge;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_point_cloud_map_merge_node");
  ros::NodeHandle nh_;

  std::string NAMESPACE_NAME;
  std::string ROBOT_NAME;
  std::string CLOUD_TYPE;
  std::string NODE_NAME;

  NAMESPACE_NAME = ros::this_node::getNamespace();
  NODE_NAME = ros::this_node::getName();
  nh_.param(NODE_NAME + "/robot_name", ROBOT_NAME, ROBOT_NAME);
  nh_.param(NODE_NAME + "/cloud_type", CLOUD_TYPE, CLOUD_TYPE);

  PointCloudFuser pclf;
  ros::Subscriber pclf_sub_5_ = nh_.subscribe("/A51/octomap_point_cloud_" + CLOUD_TYPE, 1, &PointCloudFuser::pclCallback_1, &pclf);
  ros::Subscriber pclf_sub_6_ = nh_.subscribe("/A52/octomap_point_cloud_" + CLOUD_TYPE, 1, &PointCloudFuser::pclCallback_2, &pclf);
  ros::Publisher pclf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pclf_" + CLOUD_TYPE, 1000);

  ros::Rate loop_rate(0.2);
  
  long int cnt;

  while(ros::ok())
  {
	  cnt ++;

    if(cnt == 1)
    {
      cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
      cout << "NAMESPACE_NAME: " << NAMESPACE_NAME << endl;
      cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
      cout << "NODE_NAME: " << NODE_NAME << endl;
      cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
      cout << "ROBOT_NAME: " << ROBOT_NAME << endl;
      cout << "CLOUD_TYPE: " << CLOUD_TYPE << endl;
      cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
    }

    pclf.fusePCL(pclf.sen_PCL_1, pclf.sen_PCL_2);
	  
    // Publish merged sensor_msgs::PointCloud2
    pclf_pub_.publish(pclf.sen_PCL_merge);
    
    loop_rate.sleep();
    ros::spinOnce();
  }
}

