#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

int fd;
void motor_output_callback(const std_msgs::Float64::ConstPtr& msg) {
  //ROS_INFO("Received motor output message: %f", msg->data);
  char toSend[256];
  sprintf(toSend, "%f\n", msg->data);
  int len;
  for (len = 0; len < 256 && (toSend[len] != 0); len++) ;
  write(fd, toSend, len);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "driverstation");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float64>("potentiometer_val", 1);
  ros::Subscriber sub = n.subscribe("motor_output", 1, motor_output_callback);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  char* portname = "/dev/ttyACM0";
  char buf[256];

  /* Open the file descriptor in non-blocking mode */
  fd = open(portname, O_RDWR | O_NOCTTY);

  /* Set up the control structure */
  struct termios toptions;

  /* Get currently set options for the tty */
  tcgetattr(fd, &toptions);

  /* Set custom options */

  /* 9600 baud */
  cfsetispeed(&toptions, B9600);
  cfsetospeed(&toptions, B9600);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;

  /* wait for 12 characters to come in before read returns */
  /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
  /* CHARACTERS HAVE COME IN! */
  /* no minimum time to wait before read returns */
  //toptions.c_cc[VTIME] = 0;

  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);

  /* Wait for the Arduino to reset */
  usleep(1000*1000);

  char tempBuf[256];
  int bufLen = 0;

  while (ros::ok()) {
    int ret = read(fd, buf, 1);
    if (ret > 0) {
      if (buf[0] == '\n') {
        float d = atof(tempBuf);
        //ROS_INFO("Read number: %f", d);
        bufLen = 0;

        std_msgs::Float64 toPub;
        toPub.data = d;
        pub.publish(toPub);
      } else {
        tempBuf[bufLen] = buf[0];
        bufLen++;
      }
    }
  }
}
