#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// #include 

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "joint_trajectory_test");

  ros::NodeHandle nh;

  auto joint_traj_pub =  nh.advertise<trajectory_msgs::JointTrajectory>("/irb2600/joint_path_command", 10);  

  trajectory_msgs::JointTrajectory joint_traj_msg;

  trajectory_msgs::JointTrajectoryPoint point;
  std::fill(point.positions.begin(), point.positions.end(), 0.0);
  // point
  joint_traj_msg.points.push_back(point);
  while(ros::ok())
  joint_traj_pub.publish(joint_traj_msg);
  
  ros::spin();
  ros::shutdown();
  return 0;
}
