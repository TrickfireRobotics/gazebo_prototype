#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Drivebase : public hardware_interface::RobotHW
{
public:
  double potVal;
  ros::Publisher *pub_fl, *pub_fr, *pub_rl, *pub_rr;

  Drivebase()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle wheel_fl_state_handle("wheel_fl_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(wheel_fl_state_handle);
    hardware_interface::JointStateHandle wheel_fr_state_handle("wheel_fr_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(wheel_fr_state_handle);
    hardware_interface::JointStateHandle wheel_rl_state_handle("wheel_rl_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(wheel_rl_state_handle);
    hardware_interface::JointStateHandle wheel_rr_state_handle("wheel_rr_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(wheel_rr_state_handle);

    registerInterface(&jnt_state_interface);
    // connect and register the joint position interface
    hardware_interface::JointHandle wheel_fl_eff_handle(jnt_state_interface.getHandle("wheel_fl_joint"), &cmd[0]);
    jnt_eff_interface.registerHandle(wheel_fl_eff_handle);
    hardware_interface::JointHandle wheel_fr_eff_handle(jnt_state_interface.getHandle("wheel_fr_joint"), &cmd[1]);
    jnt_eff_interface.registerHandle(wheel_fr_eff_handle);
    hardware_interface::JointHandle wheel_rl_eff_handle(jnt_state_interface.getHandle("wheel_rl_joint"), &cmd[2]);
    jnt_eff_interface.registerHandle(wheel_rl_eff_handle);
    hardware_interface::JointHandle wheel_rr_eff_handle(jnt_state_interface.getHandle("wheel_rr_joint"), &cmd[3]);
    jnt_eff_interface.registerHandle(wheel_rr_eff_handle);

    registerInterface(&jnt_eff_interface);
  }

  void read() {
    ROS_INFO("Pot val: %f", (potVal * 2.0) - 1.0);
    //vel[0] = (potVal * 2.0) - 1.0;
    vel[0] = 0.0;
    vel[1] = 0.0;
    vel[2] = 0.0;
    vel[3] = 0.0;
  }

  void write() {
    ROS_INFO("Current command: %f, %f, %f, %f", cmd[0], cmd[1], cmd[2], cmd[3]);
    std_msgs::Float64 fl, fr, rl, rr;
    fl.data = cmd[0];
    fr.data = cmd[1];
    rl.data = cmd[2];
    rr.data = cmd[3];
    pub_fl->publish(fl);
    pub_fr->publish(fr);
    pub_rl->publish(rl);
    pub_rr->publish(rr);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  double cmd[4];
  double pos[4];
  double vel[4];
  double eff[4];
};
