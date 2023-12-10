#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.hpp>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{	
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm");

// Setup

  static const std::string PLANNING_GROUP = "arm";

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

  // The :move_group_interface:`MoveGroup` class can be easily   
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  move_group->setEndEffectorLink("Link_6_1");
  geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
  
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);
  RCLCPP_INFO(node->get_logger(), "x position: %f", current_pose.pose.position.x);
  RCLCPP_INFO(node->get_logger(), "y position: %f", current_pose.pose.position.y);
  RCLCPP_INFO(node->get_logger(), "z position: %f", current_pose.pose.position.z);
  RCLCPP_INFO(node->get_logger(), "x orientation: %f", current_pose.pose.orientation.x);
  RCLCPP_INFO(node->get_logger(), "y orientation: %f", current_pose.pose.orientation.y);
  RCLCPP_INFO(node->get_logger(), "z orientation: %f", current_pose.pose.orientation.z);
  RCLCPP_INFO(node->get_logger(), "w orientation: %f", current_pose.pose.orientation.w);

  // Debugging and visualizing objects, robots and trajectories in Rviz
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "odom");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0; 
  visual_tools.publishText(text_pose, "MoveGroupInterface Moveo Demo", rvt::WHITE, rvt::XLARGE);
  
  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str());

  //Planning to a Pose Goal

  // Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal
  geometry_msgs::msg::Pose target_pose1;

  // Default pose
  target_pose1.position.x = 0.120679;
  target_pose1.position.y = 0.072992;
  target_pose1.position.z = 0.569166;
  target_pose1.orientation.x = -0.386473;
  target_pose1.orientation.y =  -0.418023;
  target_pose1.orientation.z = -0.760978;
  target_pose1.orientation.w = 0.311139;

  move_group->setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(node->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // We can visualize the plan as a line with markers in Rviz.
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group->move();

  rclcpp::shutdown();  
  return 0;
}