/**
 * @file Collision check service for a manipulator
 * @author Peijie Xu (peijiexu@usc.edu)
 * @brief (1) edit launch file to load manipulator's planning_context.launch
 *            file into robot_description (a ROS server)
 *        (2) choose move group, in this file the default group is
 *            "manipulator", need to be adapted according to diversive
 *            situations
 * @version 0.1
 * @date 2022-10-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <abb_dev/ABBCollisionCheck.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>

template <typename T>
void print_vector(const std::vector<T>& vec) {
  auto iter = vec.begin();
  while (iter != vec.end()) std::cout << *(iter++) << " ";
  std::cout << std::endl;
}

class CollisionChecker {
 public:
  CollisionChecker() {
    robot_model_loader =
        new robot_model_loader::RobotModelLoader("robot_description");

    planning_scene =
        new planning_scene::PlanningScene(robot_model_loader->getModel());

    current_state = &planning_scene->getCurrentStateNonConst();
  }

  ~CollisionChecker() {
    delete robot_model_loader;
    delete planning_scene;
  }

  bool check_collision(std::vector<double>& target_joint_values) {
    joint_model_group = current_state->getJointModelGroup("manipulator");

    current_state->setJointGroupPositions(joint_model_group,
                                          target_joint_values);
    acm = planning_scene->getAllowedCollisionMatrix();
    collision_result.clear();
    planning_scene->checkCollision(collision_request, collision_result,
                                   *current_state, acm);
    return collision_result.collision;
  }

 private:
  robot_model_loader::RobotModelLoader* robot_model_loader;
  planning_scene::PlanningScene* planning_scene;
  moveit::core::RobotState* current_state;

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  const moveit::core::JointModelGroup* joint_model_group;

  collision_detection::AllowedCollisionMatrix acm;
};

CollisionChecker* cc;

bool cc_callback(abb_dev::ABBCollisionCheck::Request& req,
                 abb_dev::ABBCollisionCheck::Response& res) {
  // std::vector<double> target_joint_values = {0.0, 0.0, 0.0, 2.9, 0.0, 1.4};
  // res.collided = cc->check_collision(target_joint_values);
  res.collided = cc->check_collision(req.joint_state);
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "abb_collision_check_node");
  ros::NodeHandle nh;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  cc = new CollisionChecker();

  // ROS_INFO_STREAM("Test: Current state is "
  //                 << (cc.check_collision(target_joint_values) ? "in" : "not
  //                 in")
  //                 << " self collision");

  // const std::vector<std::string>& joint_names =
  // joint_model_group->getVariableNames(); std::vector<double> joint_values;
  // current_state.copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i) {
  //   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }

  ros::ServiceServer cc_service =
      nh.advertiseService("/cam/irb2600/CollisionCheck", cc_callback);

  ros::spin();

  ros::shutdown();
  return 0;
}
