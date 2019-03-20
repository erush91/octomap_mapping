#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

namespace convert_oct_pcl
{
  class ConvertOctomapToPCL
  {
  public:
    
    std::string NAMESPACE_NAME;
    std::string CLOUD_TYPE;
    std::string NODE_NAME;

    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef octomap::OcTree OcTreeT;

    ConvertOctomapToPCL(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    virtual ~ConvertOctomapToPCL(){};
    virtual void octomapCallback(const octomap_msgs::Octomap::ConstPtr msg);

    ros::Subscriber m_octoMapBinarySub;
    ros::Publisher  m_pointCloudOccupiedPub, m_pointCloudFreePub;

    ros::NodeHandle m_nh;
    OcTreeT* m_octree;

    std::string m_worldFrameId; // the map frame
    ros::Time rostime;

    bool m_latchedTopics;
    bool m_publishFreeSpace;
    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;   

    bool receivedOctomap = 0;
      
    sensor_msgs::PointCloud2 cloud_occupied;
    sensor_msgs::PointCloud2 cloud_free;
  
    pcl::PointCloud<PCLPoint> pclCloudOccupied;
    pcl::PointCloud<PCLPoint> pclCloudFree;

  };
}

