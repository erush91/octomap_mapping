///////////////////////////////////////////////////////////////////////////////
// File Name : octomap_to_pcl
// Purpose : Provide octomapCallback, which converts an octomap to a pcl
///////////////////////////////////////////////////////////////////////////////
// Changelog :
// Date           % Name       %   Reason
// 03 / 12 / 2019 % Gene Rush  %   Created - Added code and comments
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

#include "octomap_to_pcl.h"

using std::cout;
using std::endl;

namespace convert_oct_pcl{

ConvertOctomapToPCL::ConvertOctomapToPCL(ros::NodeHandle private_nh_)
: m_nh(),
  m_octree(NULL),
  m_worldFrameId("/map"),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0)
{
  
  ros::NodeHandle private_nh(private_nh_);
  
  NAMESPACE_NAME = ros::this_node::getNamespace();
  NODE_NAME = ros::this_node::getName();

  private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);
  private_nh.param("latch", m_latchedTopics, m_latchedTopics);
  
  m_octoMapBinarySub = m_nh.subscribe(NAMESPACE_NAME + "/octomap_binary_throttle", 1, &ConvertOctomapToPCL::octomapCallback, this);
  m_pointCloudOccupiedPub = m_nh.advertise<sensor_msgs::PointCloud2>(NAMESPACE_NAME + "/pcl_local_occupied", 1);
  m_pointCloudFreePub = m_nh.advertise<sensor_msgs::PointCloud2>(NAMESPACE_NAME + "/pcl_local_free", 1);
}

  void ConvertOctomapToPCL::octomapCallback(const octomap_msgs::Octomap::ConstPtr msg)
  {
    // ROS_INFO("Getting OctoMap message...");
    if (!receivedOctomap) receivedOctomap = 1;

    // Free/Allocate the tree memory
    // ROS_INFO("Converting Octomap msg to AbstractOcTree...");
    // octomap::AbstractOcTree* abstree = new octomap::AbstractOcTree(msg->resolution);
    // abstree = octomap_msgs::binaryMsgToMap(*msg); // OcTree object for storing Octomap data.
    // ROS_INFO("Octomap converted to AbstractOcTree.");
    octomap::OcTree* m_octree = new octomap::OcTree(msg->resolution);
    m_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
    // ROS_INFO("AbstractOcTree cast into OcTree.");

    double size, value;
    float lower_corner[3];
    int depth, width;
    int lowest_depth = (int)m_octree->getTreeDepth();
    float voxel_size = 0.1; // GENE THIS IS IMPORTANT!!!
    unsigned m_treeDepth = m_octree->getTreeDepth();
    unsigned m_maxTreeDepth = m_treeDepth;
    long int cnt;
    
    // Traverse all leafs in the tree:
    for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
        end = m_octree->end(); it != end; ++it)
    {
      cnt ++;
      depth = (int)it.getDepth();
      size = it.getSize();
      // Extract free leaflet  information
      if (m_octree->isNodeOccupied(*it))
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        // Leaflet is at the lowest depth
        if (depth == lowest_depth)
        {
          pclCloudOccupied.push_back(PCLPoint(x, y, z));
        }
        // Leaflet is at a higher depth
        else
        {
          // Fill in all the voxels internal to the leaf
          int width = (int)std::pow(2.0, (double)(lowest_depth-depth));
          lower_corner[0] = x - size/2.0 + voxel_size/2.0;
          lower_corner[1] = y - size/2.0 + voxel_size/2.0;
          lower_corner[2] = z - size/2.0 + voxel_size/2.0;
          PCLPoint _point = PCLPoint();
          for (int i=0; i<width; i++)
          {
            _point.x = lower_corner[0] + i*voxel_size;
            for (int j=0; j<width; j++)
            {
              _point.y = lower_corner[1] + j*voxel_size;
              for (int k=0; k<width; k++)
              {
                _point.z = lower_corner[2] + k*voxel_size;
                pclCloudOccupied.push_back(_point);
              }
            }
          }
        }
      }
      // Extract free leaflet information
      else
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        // Leaflet is at the lowest depth
        if (depth == lowest_depth)
        {
          pclCloudFree.push_back(PCLPoint(x, y, z));
        }
        // Leaflet is at a higher depth
        else
        {
          // Fill in all the voxels internal to the leaf
          int width = (int)std::pow(2.0, (double)(lowest_depth-depth));
          lower_corner[0] = x - size/2.0 + voxel_size/2.0;
          lower_corner[1] = y - size/2.0 + voxel_size/2.0;
          lower_corner[2] = z - size/2.0 + voxel_size/2.0;
          PCLPoint _point = PCLPoint();
          for (int i=0; i<width; i++)
          {
            _point.x = lower_corner[0] + i*voxel_size;
            for (int j=0; j<width; j++)
            {
              _point.y = lower_corner[1] + j*voxel_size;
              for (int k=0; k<width; k++)
              {
                _point.z = lower_corner[2] + k*voxel_size;
                pclCloudFree.push_back(_point);
              }
            }
          }
        }
      }
    }

    // Convert from pcl:PCLPointCloud2 to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud_occupied;
    pcl::toROSMsg (pclCloudOccupied, cloud_occupied);
    cloud_occupied.header.frame_id = "world";
    rostime = ros::Time::now();
    cloud_occupied.header.stamp = rostime;

    // Convert from pcl:PCLPointCloud2 to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud_free;
    pcl::toROSMsg (pclCloudFree, cloud_free);
    cloud_free.header.frame_id = "world";
    rostime = ros::Time::now();
    cloud_free.header.stamp = rostime;

    // Publish merged sensor_msgs::PointCloud2
    m_pointCloudOccupiedPub.publish(cloud_occupied);
    m_pointCloudFreePub.publish(cloud_free);
    ROS_INFO("octomap_to_pcl done");
  }
}
