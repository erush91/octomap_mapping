#include "ros/ros.h"
#include "concat_pcl/AddTwoInts.h"

bool add(concat_pcl::AddTwoInts::Request  &req,
         concat_pcl::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fuse_point_clouds_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("fuse_point_clouds", add);
  ROS_INFO("Ready to fuse point clouds.");
  ros::spin();

  return 0;
}
