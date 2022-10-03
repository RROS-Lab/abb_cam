#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace cam {
trajectory_msgs::JointTrajectory joint_traj_builder() {
  trajectory_msgs::JointTrajectory joint_traj_msg;
  joint_traj_msg.header.frame_id = "base_link";
  joint_traj_msg.joint_names.insert(
      joint_traj_msg.joint_names.begin(),
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
  return joint_traj_msg;
}
}  // namespace cam

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joint_trajectory_test");

  ros::NodeHandle nh;

  auto joint_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/irb2600/joint_path_command", 10);

  auto joint_traj_msg = cam::joint_traj_builder();

  trajectory_msgs::JointTrajectoryPoint point;
  std::fill(point.positions.begin(), point.positions.end(), 0.0);
  point.positions.insert(
    point.positions.begin(),
    {1.0, 0.13, 0.38, 4.91, -1.73, -5.89}
  );
  

  // point 0
  joint_traj_msg.points.push_back(point);

  // point 1
  *point.positions.begin() = 0.9;
  point.time_from_start.sec = 1;
  joint_traj_msg.points.push_back(point);

  // point 2
  *point.positions.begin() = 1.1;
  point.time_from_start.sec = 3;
  joint_traj_msg.points.push_back(point);

  std::cout << "press Enter to continue:"<<std::flush;
  getchar();

  
  joint_traj_pub.publish(joint_traj_msg);

  // ros::spin();
  ros::shutdown();
  return 0;
}
