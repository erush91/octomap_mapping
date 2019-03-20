#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



void poseCallback(const nav_msgs::Odometry msg){
  static tf::TransformBroadcaster br;

  tf::Transform T_world_compass(
    tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
    tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
  );

  tf::Transform T_huskybase_compass( tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0) );

  tf::Transform T_world_huskybase = T_world_compass*T_huskybase_compass.inverse();

  //br.sendTransform(tf::StampedTransform(T_world_huskybase, ros::Time::now(), "world", "SubT"));
  br.sendTransform(tf::StampedTransform(T_world_huskybase, ros::Time::now(), "world", "SubT_vehicle"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

//  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/odom", 10, &poseCallback);

  ros::spin();
  return 0;
};
