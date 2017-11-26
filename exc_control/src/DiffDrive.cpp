#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;

ros::Publisher *fl, *fr, *rl, *rr;
void drive_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Steering:
    // -1 <-- 0 --> +1

    // Axes:
    // X of linear for drive, Z of angular for turning
    double l = 0.0, r = 0.0;
    std_msgs::Float64 left, right;
    left.data = 0.0;
    right.data = 0.0;

    left.data += msg->linear.x;
    right.data += msg->linear.x;

    left.data += msg->angular.z;
    right.data -= msg->angular.z;

    left.data = max(-1.0, min(left.data, 1.0));
    right.data = max(-1.0, min(right.data, 1.0));

    fl->publish(left);
    rl->publish(left);
    fr->publish(right);
    rr->publish(right);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "driverstation");
    ros::NodeHandle n;
    ros::Publisher _fl = n.advertise<std_msgs::Float64>("/exc/wheel_fl_velocity_controller/command", 1);
    ros::Publisher _fr = n.advertise<std_msgs::Float64>("/exc/wheel_fr_velocity_controller/command", 1);
    ros::Publisher _rl = n.advertise<std_msgs::Float64>("/exc/wheel_rl_velocity_controller/command", 1);
    ros::Publisher _rr = n.advertise<std_msgs::Float64>("/exc/wheel_rr_velocity_controller/command", 1);
    fl = &_fl;
    fr = &_fr;
    rl = &_rl;
    rr = &_rr;

    ros::Subscriber sub = n.subscribe("cmd_vel", 1, drive_callback);

    ros::spin();
}