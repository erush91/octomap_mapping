#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



void poseCallback(const nav_msgs::Odometry msg){
  static tf::TransformBroadcaster br;

  // double roll, pitch, yaw;

  // tf::Quaternion tf_quat;
  // tf::quaternionMsgToTF(msg.pose.pose.orientation, tf_quat);
  // tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);


  tf::Transform T_world_jackal0(
    tf::Quaternion(0,0,0,1),
    tf::Vector3(0,0,0)
  );

  tf::Transform T_jackal_compass(
    tf::Quaternion(.7071,.7071,0,0),
    //tf::Quaternion(1,0,0,0),
    tf::Vector3(0,0,0)
  );

  tf::Transform T_compass0_compass(
    tf::Quaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
    tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
  );

  tf::Transform T_world_jackal = ((T_world_jackal0*T_jackal_compass)*T_compass0_compass)*T_jackal_compass.inverse();


  //tf::Transform T_subt_compass( tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0) );

  //tf::Transform T_world_subt = T_world_compass*T_subt_compass.inverse();

  br.sendTransform(tf::StampedTransform(T_world_jackal, ros::Time::now(), "world", "jackal"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/compass_pose", 10, &poseCallback); // /odom


  ros::spin();
  return 0;
};
