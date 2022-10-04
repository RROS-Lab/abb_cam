#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
// #include "/* header */"
// 

template <typename T>
void print_vector(const std::vector<T> & vec){
  auto iter = vec.begin();
  while (iter != vec.end()) std::cout << *(iter++) << " ";
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "abb_collision_check_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  planning_scene::PlanningScene planning_scene(kinematic_model);
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;


  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("manipulator");
  std::vector<double> target_joint_values = { 0.0, 0.0, 0.0, 2.9, 0.0, 1.4};
  current_state.setJointGroupPositions(joint_model_group, target_joint_values);


  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
  ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");


  // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  // std::vector<double> joint_values;
  // current_state.copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i) {
  //   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }
  
  
  ros::shutdown();
  return 0;
}
