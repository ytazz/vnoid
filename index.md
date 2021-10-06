![top image](fig/robot.png "Top Image")

## Aim of this challenge
The aim fo this challenge is to facilitate cutting-edge research on humanoid robotics
 (motion planning, whole-body control, environmental sensing, etc...)
 by sharing knowledge and technical resources in the research community and
 encouraging students and researchers to tackle challenging problems through competition.

Several simulation-based humanoid robot competitions have been held in the past.
Compared to these events, This challenge is aiming at exploring the potential ability of humanoid robots to
perform acrobatic movements.


## Important Dates

- Website opening: early October 2021
- Preliminary challenge: December 2021, online
- Main challenge: March 2022, online

## Challenge Theme

The goal of the challenge is to make the robot traverse the athletic field shown in the figure below
 from the starting position to the goal.

![field](fig/field.png "Athletics Field")

## Rules

Choreonoid (<choreonoid.org>) is used for the simulation environment of this challenge.
Each team designs a robot model and a controller that can be loaded and run on Choreonoid.

It also implements the controller of the robot using the SimpleController interface of Choreonoid.

### Creating a robot model

- The model is recommended to be written in the .body format.
- The number of joints must not exceed 50.
- The type of joints may be revolute or prismatic.
- The total mass of the robot must be between 50-100 [kg].
- At the initial posture, the robot must be inside the box of X x Y x Z = 70cm x 70cm x 200cm.

### Implementing a controller

- The controller should be implemented as a SimpleController of Choreonoid.
- The main controller could be implemented using some other framework such as ROS,
  and exchange information between Choreonoid over a communication bridge.

#### Actuation
- The actuation mode of each joint may be either Velocity or Torque.
  Torque mode is recommended for stable simulation.
- Only joints can be directly actuated.
  Directly actuating the links (the rigid bodes) is not permitted in the trial (use it for test purpose only).
  
#### Sensing
- Various sensor nodes supported by Choreonoid such as ForceSensor, RateGyroSensor, and AccelerationSensor, may be used.
  The number of sensors is not limited.
- The position and velocity of joints can be retrieved through the SimpleControllerIO interface.
- Retrieving the absolute position and orientation of links (rigid bodies) is also permitted
  (there is no need for position/posture estimation).

### Simulator setting
- The AISTsimulator physics engine for simulation.
- Switching to other physics engines. or changing the parameters of the AISTsimulator is not permitted in the trial.
- A sample .cnoid project file is provided.
  Use the simulator setting described in this file for the trial.

### Operating the robot
- The robot can be operated with an input device, typically a joystick.
- The operator can operate the robot while monitoring the robot on the 3D view of Choreonoid.

### Evaluation
- The quality of the trial is scored based on the following criteria:

#### Goal reaching
- The goal and several intermediate checkpoints are specified in the field. They are shown in different colors.
  Score is assigned based on the furthest checkpoint (of the goal) reached during the trial.
  
#### Realtime-ness
- Score is assigned based on the ratio of simulation-time versus computation-time.

#### Subjective evaluation
- 


## Quick Start



## Asking Questions



