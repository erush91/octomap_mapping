#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu msg){
  static tf::TransformBroadcaster br;

  tf::Transform T_world_imu_orientation(
    tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    tf::Vector3(0.0, 0.0, 0.0)
  );

  br.sendTransform(tf::StampedTransform(T_world_imu_orientation, ros::Time::now(), "world", "imu_orientation"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_orientation_tf_broadcaster");

//  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/imu_raw", 10, &imuCallback);

  ros::spin();
  return 0;
};
