#include <SFML/Graphics.hpp>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

using namespace std;

float armRot = 0.0f;

void handle_drive(ros::Publisher vel_pub) {
  geometry_msgs::Twist twist;

  float drive = 0.f, rot = 0.f;

  if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
    drive += 1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
    drive -= 1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
    rot -= 1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
    rot += 1.f;
  }
  twist.linear.x = drive * 1.f;
  twist.angular.z = -rot * 1.f;
  vel_pub.publish(twist);
}

void handle_turntable(ros::Publisher turn_pub) {
  // Two keys, < and >, will rotate the arm by 15 degrees each press
  std_msgs::Float64 turn;
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Period)) { // Rotate right, decrease angle
    armRot -= M_PI / 6;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Comma)) { // Rotate left, increase angle
    armRot += M_PI / 6;
  }

  turn.data = armRot;

  turn_pub.publish(turn);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driverstation");
  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher turn_pub = n.advertise<std_msgs::Float64>("/turntable_position_controller/command", 1);

  sf::RenderWindow window(sf::VideoMode(800, 600), "My window");

  while (window.isOpen() && ros::ok())
  {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event;
    while (window.pollEvent(event))
    {
      switch (event.type) {
        case sf::Event::Closed: {
          window.close();
        }
        case sf::Event::KeyPressed:
        case sf::Event::KeyReleased: {
          handle_drive(vel_pub);
          handle_turntable(turn_pub);
        }
        default: break;
      }
    }

    window.clear(sf::Color::Black);

    window.display();
    ros::spinOnce();
    window.setFramerateLimit(30);
  }

  return 0;
}
