#include "Drivebase.cpp"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

Drivebase robot;

void pot_callback(const std_msgs::Float64::ConstPtr& msg) {
  //ROS_INFO("Received motor output message: %f", msg->data);
  robot.potVal = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driverstation");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("potentiometer_val", 1, pot_callback);
  ros::Publisher pub_fl = n.advertise<std_msgs::Float64>("/motor_fl", 1);
  ros::Publisher pub_fr = n.advertise<std_msgs::Float64>("/motor_fr", 1);
  ros::Publisher pub_rl = n.advertise<std_msgs::Float64>("/motor_rl", 1);
  ros::Publisher pub_rr = n.advertise<std_msgs::Float64>("/motor_rr", 1);
  robot.pub_fl = &pub_fl;
  robot.pub_fr = &pub_fr;
  robot.pub_rl = &pub_rl;
  robot.pub_rr = &pub_rr;

  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);

  spinner.start();

  ros::Time then = ros::Time::now();
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();

    robot.read();
    cm.update(now, now - then);
    robot.write();

    then = now;
    rate.sleep();
  }
}
