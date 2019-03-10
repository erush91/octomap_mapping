#include "ros/ros.h"
#include "concat_pcl/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pclf_client");
  if (argc != 3)
  {
    ROS_INFO("usage: fuse point clouds pclA plcB");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<concat_pcl::AddTwoInts>("fuse_point_clouds");
  concat_pcl::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service fuse_point_clouds");
    return 1;
  }

  return 0;
}
