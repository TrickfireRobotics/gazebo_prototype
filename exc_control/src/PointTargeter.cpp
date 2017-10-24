#include <SFML/Graphics.hpp>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <math.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"

using namespace std;

tf2_ros::Buffer * tfBuffer;
tf2_ros::TransformListener * tfListener;

ros::Publisher vel_pub;
ros::Publisher turn_pub;
ros::Publisher lower_arm_pub;
ros::Publisher upper_arm_pub;

void pointCallback(const geometry_msgs::PointStampedConstPtr& msg) {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tfBuffer->lookupTransform("turntable_static", "odom", ros::Time(0));
    geometry_msgs::PointStamped pt;
    tf2::doTransform(*msg, pt, tf);

    float theta = atan2(pt.point.y, pt.point.x);
    std_msgs::Float64 turn;
    turn.data = theta;
    turn_pub.publish(turn);

  } catch (tf2::TransformException ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointtargeter");
  ros::NodeHandle n;

  tf2_ros::Buffer tfB;
  tf2_ros::TransformListener tfL(tfB);
  tfBuffer = &tfB;
  tfListener = &tfL;

  vel_pub = n.advertise<geometry_msgs::Twist>("/exc/cmd_vel", 1);
  turn_pub = n.advertise<std_msgs::Float64>("exc/turntable_position_controller/command", 1);
  lower_arm_pub = n.advertise<std_msgs::Float64>("exc/lower_arm_position_controller/command", 1);
  upper_arm_pub = n.advertise<std_msgs::Float64>("exc/upper_arm_position_controller/command", 1);
  ros::Subscriber sub = n.subscribe("/clicked_point", 1, pointCallback);

  // Test MoveIt setup
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("god");
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  ros::spin();

  return 0;
}