#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "jackal_odom_publisher");

  ros::NodeHandle node;

  ros::Publisher jackal_odom_pub = 
    node.advertise<nav_msgs::Odometry>("jackal_odom", 10);

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  odom_msg.header.frame_id = "world"; 
  ros::Rate rate(50.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/jackal",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = transform.getOrigin().x();
    odom_msg.pose.pose.position.y = transform.getOrigin().y();
    odom_msg.pose.pose.position.z = transform.getOrigin().z()+.25;
    tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);

    jackal_odom_pub.publish(odom_msg);

    rate.sleep();
  }
  return 0;
};