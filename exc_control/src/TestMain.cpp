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
  ros::Publisher pub = n.advertise<std_msgs::Float64>("/motor_out", 1);
  robot.pub = &pub;

  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);

  spinner.start();

  ros::Time then = ros::Time::now();
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    ROS_INFO("Doing the thing!");
    const ros::Time now = ros::Time::now();

    robot.read();
    cm.update(now, now - then);
    robot.write();

    then = now;
    rate.sleep();
  }
}
