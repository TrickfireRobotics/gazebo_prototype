# TrickFire Robotics 2017
## Gazebo Simulation Prototype
A robot simulation in Gazebo for use in prototyping various concepts. This takes the core of a simple skid-steer robot (designed to simulate tank treads as treads aren't supported in Gazebo) and then allows all of our team members to add on to the chassis with their own prototype additions such as Kinects, IMUs, lidars, or anything else we feel needs to be tested.

### Required libraries
 * Core ROS libraries
 * ROS Gazebo libraries + Gazebo itself (see tutorials linked below which cover this)
 * SFML (installing from source seems to work better, but I've also gotten just `sudo apt-get install libsfml-dev` to work)
If you have any build issues please by all means mention them to Adam (me), I'll do my best to address them.

## Branching Policy
No commits will be made to `master` except for changes that change the base robot that would be useful for all prototypes. For any feature you prototype (the digging arm, a Lidar, a Kinect, etc.) create a branch for it with a sensible, self-explanatory name. All prototypes will remain separate from each other and from `master` unless two prototypes feel it would be useful to merge together. Feel free to clone other prototype's branches to test them out, but do not modify them without permission from the prototype maintainer. If you're just messing around for fun keep it to your local machine, but if you think the changes constitute a notable feature/prototype development, go ahead and make a separate (also sensibly named) branch for it or let the prototyper know to implement it.

## Where to Go for Help
Feel free to ask Adam for help with Gazebo or any of the base files. There is also a good getting started tutorial for installing Gazebo and basic model definitions at http://gazebosim.org/tutorials?cat=connect_ros. Also, check out the "RRBot" (Reverse Revolute Bot, or a double pendulum) example for a relatively easy to understand sample project here: http://gazebosim.org/tutorials/?tut=ros_urdf (this is part of the previously mentioned tutorial series, but to be honest the actual sample was much more useful than the tutorials by themselves for me). 

## ROS Package Summary
This repository is a higher-level folder of 3 separate but related ROS packages.
### exc_description
This contains the model definition of the robot. This is where you can define and change how the robot looks, various physical properties such as inertia and mass, and add sensors such as cameras, Kinects, IMUS, Lidars, etc. The model files can be found in the `urdf` directory. There is a launch file to view the model by itself (without the simulator, physics etc.)

### exc_gazebo
This contains the Gazebo world and the launch file to bring it up and spawn a robot. The world file (currently empty) is in the `world` folder, and the launch file to start Gazebo, spawn the robot, and also start a couple other miscallaneous processing nodes (tf state publishers, joint controllers, etc.) is in the `launch` folder.

### exc_control
This contains anything needed to control the robot. Currently there is a very simple driver station application in the `src` folder. It's a black window and simply uses WASD controls to drive the robot. This can be relatively easily expanded on with more keybinds, mouse, or joystick input.
