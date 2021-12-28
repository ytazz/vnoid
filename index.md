![top image](fig/robot.png "Top Image")

## Aim of this challenge

The aim of this challenge is to provide students and researchers in the humanoid robotics research community
 with a common development platform and an opportunity for competition and
 thereby facilitate cutting-edge research in this field
 through the sharing of knowledge, ideas, and technical resources.

Compared to several simulation-based humanoid robot competitions held in the past (e.g., JVRC, DRC, WRS, Robocup),
 the major uniqueness of this challenge is that it focuses of the potential ability of humanoid robots to perform acrobatic movements.


## Important Dates

- Website opening: early October 2021
- Application deadline: **15 November 2021**  still open for later application
- Preliminary challenge: **25 December 2021**, online, finished.
- Main challenge: **12 March 2022**, online

### List of Teams

Listed in the order of application.
Results of preliminary challenge are listed (Dec 28, 2021).

|  Team name        |  Robot name | Reached Landmark  | Simulation Time | Realtime-factor |
| ----              | ----        | ----              | ----            | ----            |
| MA1               | Uriborg Mk1 | Green             | 0:25            | 1.15            |
| ssr-act           |             |                   |                 |                 |
| FisMa	            |             |                   |                 |                 |
| Legendless  	    |             | Orange (Top stair)| 3:21            |                 |
| solver	        |             | Blue              |                 |                 |
| 水内研究室仮配属1 | First Gear  | Green             | 0:55            | 2.29            |
| IRL-Meiji	        | Meiped-HVAC |                   |                 |                 |
| 落合重工          |             |                   |                 |                 |
| prime4294967279	|             | Green             | 0:09            | 1.17            |
| Team Jaxon        | Jaxon       | Red (Goal)        | 2:32            | 1.48            |
| 水内研究室仮配属  | minimum man | Blue              |                 | 1.00            |


## Challenge Theme

The goal of the challenge is to make the robot traverse the athletic field shown in the figure below
 from the starting position to the goal, either by teleoperation, automatic control, or combination of both.

![field](fig/field.png "Athletics Field")

## Regulation

See [here](https://drive.google.com/file/d/1gUBYM62HW0czXO8-PiJqJn6ycbd-Wtij/view?usp=sharing)
for detailed regulation (Japanese only).

[Choreonoid](choreonoid.org) is used for the simulation environment of this challenge.
Each team designs a robot model and a controller that can be loaded and run on Choreonoid.

### Creating a robot model

- The model is recommended to be written in the .body format.
- The number of joints must not exceed 50.
- The type of joints may be revolute or prismatic.
- The total mass of the robot must be 40[kg] at the minimum.
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
- Various sensor nodes supported by Choreonoid such as ForceSensor, RateGyroSensor, AccelerationSensor, and RangeSensor, may be used.
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
The quality of the trial is evaluated based on the following criteria:

#### Goal reaching
- The goal and several intermediate checkpoints are specified in the field. They are shown in different colors.
  Whether the robot could reach each checkpoint during the trial is recorded.
  
#### Speed
- Simulation time elapsed before reaching the each checkpoint is recorded.
  
#### Realtime-ness
- The ratio of simulation-time and computation-time is recorded to evaluate how much the controller violates the realtime constraint.


## Quick Start
- See [here](https://ytazz.github.io/vnoid/build_sample.html) for how to build and run the sample.

## How to Apply
- Fill in and send your application information from [Google Forms](https://docs.google.com/forms/d/e/1FAIpQLSfSJpFjd2HKMdh3X6ul73qPH_Yzn0-rUjdUMSX40r42tThosQ/viewform?usp=sf_link)

## Asking Questions
- Post your question at [https://github.com/ytazz/vnoid/discussions](https://github.com/ytazz/vnoid/discussions)


