#include "HardwareInterface.cpp"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

HardwareInterface hw;

void pot_callback(const std_msgs::Float64::ConstPtr& msg) {
  //ROS_INFO("Received motor output message: %f", msg->data);
  hw.potVal = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_launcher");
  ros::NodeHandle n;
  ros::Publisher pub_fl = n.advertise<std_msgs::Float64>("/motor_fl", 1);
  ros::Publisher pub_fr = n.advertise<std_msgs::Float64>("/motor_fr", 1);
  ros::Publisher pub_rl = n.advertise<std_msgs::Float64>("/motor_rl", 1);
  ros::Publisher pub_rr = n.advertise<std_msgs::Float64>("/motor_rr", 1);
  ros::Publisher pub_arm = n.advertise<std_msgs::Float64>("/motor_arm", 1);
  ros::Subscriber sub = n.subscribe("potentiometer_val", 1, pot_callback);
  hw.pub_fl = &pub_fl;
  hw.pub_fr = &pub_fr;
  hw.pub_rl = &pub_rl;
  hw.pub_rr = &pub_rr;
  hw.pub_arm = &pub_arm;

  controller_manager::ControllerManager cm(&hw);

  ros::AsyncSpinner spinner(1);

  spinner.start();

  ros::Time then = ros::Time::now();
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();

    hw.read();
    cm.update(now, now - then);
    hw.write();

    then = now;
    rate.sleep();
  }
}
