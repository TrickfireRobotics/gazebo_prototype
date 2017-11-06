#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/console.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  static const std::string PLANNING_GROUP = "arm_end";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  ROS_INFO_NAMED("test_plan", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("test_plan", "End effector: %s", move_group.getEndEffectorLink().c_str());

  // WORTH NOTING: we could possibly specify things like the scoop always
  // remaining horizontal (to avoid dumping on accident) using this code? :D
  /*moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "arm_end";
  ocm.header.frame_id = "base_link";
  const double tol = 1.0;
  ocm.absolute_x_axis_tolerance = tol;
  ocm.absolute_y_axis_tolerance = tol;
  ocm.absolute_z_axis_tolerance = tol;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);*/

  cout << "Enter target x, z, and theta (degrees): ";
  double x, z, theta;
  //cin >> x >> z >> theta;
  cout << endl;
  theta *= 3.14159265/180; // Convert to radians in place, so it's simpler later

  geometry_msgs::Pose target_pose;
  // Pointing downwards and at the specified angle
  tf::Quaternion targetAngle = tf::createQuaternionFromRPY(3.14159265, 3.14159265/2, -3.14159265/2);
  targetAngle.normalize(); // Just in case

  target_pose.orientation.x = targetAngle[0];
  target_pose.orientation.y = targetAngle[1];
  target_pose.orientation.z = targetAngle[2];
  target_pose.orientation.w = targetAngle[3];

  //target_pose.position.x = 0.5 + (x * cos(theta));
  //target_pose.position.y = x * sin(theta);
  //target_pose.position.z = z;

  target_pose.position.x = 0.5;
  target_pose.position.y = 1.5;
  target_pose.position.z = 1.0;
  ROS_INFO("Position x,y: %f, %f, %f", target_pose.position.x, target_pose.position.y, theta);
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);

  ROS_INFO_NAMED("tutorial", "Visualizing pose goal %s", success ? "" : "FAILED");

  if (success) {
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");
    ROS_INFO_NAMED("tutorial", "Moving to planned pos...");
    move_group.move();
  }
}
