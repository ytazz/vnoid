<!--
![top image](fig/robot.png "Top Image")
-->
![top image](fig/hvac2021.png "Top Image")

<iframe width="560" height="315" src="https://www.youtube.com/embed/2xOmlHx2b4U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Challenge 2023

<!--
Under construction. Information is tentative.
-->

## Aim of this challenge

The aim of this challenge is to provide students and researchers in the humanoid robotics research community
 with a common development platform and an opportunity for competition and
 thereby facilitate cutting-edge research in this field
 through the sharing of knowledge, ideas, and technical resources.

Compared to several simulation-based humanoid robot competitions held in the past (e.g., JVRC, DRC, WRS, Robocup),
 the major uniqueness of this challenge is that it focuses of the potential ability of humanoid robots to perform acrobatic movements.

## How to participate

- This challenge takes place based on participants' submissions of simulation log data.
- If you would like to participate in the challenge, please submit application information
  from the link shown below not later than the application deadline.
- After application, you will receive a link for uploading simulation log data.
- After that, you can submit simulation log data at any time during the period of challenge.
  Submitted data will be checked by the steering committee if it conforms with the regulation,
   and if qualified, will appear in the list of qualified submissions shown in this webpage.

## Important Dates of 2023

- Website opening: August
- Application deadline: T.B.D.
- Preliminary Competition Event: November 11, 2023
- Main Competition Event: March 16, 2024
- Challenge period: Until March 16, 2024

## Application form

<!--
Now being prepared.
-->

-[Here](https://docs.google.com/forms/d/e/1FAIpQLScJD0zL3r_wv2M09Hj_TMIfLwAr0-pc8-OSdCSNlDXvBhAdZg/viewform?usp=sf_link)

<!--
2022
[Here](https://docs.google.com/forms/d/e/1FAIpQLSdFkUbLTEVpoHGqvfrTeW59-84Kaf-RCNaI9FLpQUu1h0EpCQ/viewform?usp=sf_link)
-->

## Challenge Themes

### Athletics

The robot traverses the athletic field from the starting position to the goal.
Performance is evaluated by reached landmark and elapsed simulation time.

![field](fig/field_athletics2023.png "Athletics Field")

### Dance

The robot performs dancing that requires whole-body motion.
Dance performance is evaluated in terms of both artistic quality and technical quality.

![robots](fig/dance2023.png "Robot models used for dance performance")

<!--
- [Results](https://ytazz.github.io/vnoid/results_athletics2022.html)
-->

### Short track

The robot walks/runs the 20[m] short track.
Performance is evaluated by elapsed simluation time while the robot passes the start line and the goal line.

![field](fig/field_shorttrack2022.png "Short-track Field")

<!--
- [Results](https://ytazz.github.io/vnoid/results_shorttrack2022.html)
-->

<!--
### Free-style performance

The robot performs something extraordinary.
How to evaluate the quality of free-tyle performance is T.B.D.

![field](fig/field_performance2022.png "Free-style Performance Field")
-->

## Regulation

### Detailed regulation

Please see the following document for detailed regulations.
Regulations are tentative and subject to change.
- [English version] Being prepared.
- [共通レギュレーション](https://docs.google.com/document/d/1dcAGtvMtwe-3d4sP9VaLWPo771_S5pBR--lq0CypRls/edit?usp=sharing)
- [アスレチック](https://docs.google.com/document/d/1ib7GlNOn1qRKZZ-hGxog0hHFQIQzdH84j6EV4cKOJnc/edit?usp=sharing)
- [ダンス](https://docs.google.com/document/d/1AZ8EWB1Y8ITu5elp3QQBQHv5C-FL7t1HwnD2UbKIIa0/edit?usp=sharing)
- [短距離](https://docs.google.com/document/d/1hnAQm6T4lx1NdhgrTaBNoMrfIc3fg87qDnvxAWstkNM/edit?usp=sharing)

### Regulations overview

[Choreonoid](https://www.choreonoid.org/) is used for the simulation environment of this challenge.
Each team designs a robot model and a controller that can be loaded and run on Choreonoid.

#### Creating a robot model

- The model is recommended to be written in the .body format.
- The number of joints must not exceed 50.
- The type of joints may be revolute or prismatic.
- The total mass of the robot must be 40[kg] at the minimum.
- The robot in its initial posture must be inside the box of X x Y x Z = 70cm x 70cm x 200cm.

#### Implementing a controller

- The controller should basically be implemented as a SimpleController of Choreonoid.
- The main controller could be implemented using some other framework such as ROS,
  and exchange information between Choreonoid over a communication bridge.

#### Actuation
- The actuation mode of each joint may be either Velocity or Torque.
  Torque mode is recommended for stable simulation.
- Only joints can be directly actuated.
  Directly actuating the links (the rigid bodes) is not permitted (use it for test purpose only).
  
#### Sensing
- Various sensor nodes supported by Choreonoid such as ForceSensor, RateGyroSensor, AccelerationSensor, and RangeSensor, may be used.
  The number of sensors is not limited.
- The position and velocity of joints can be retrieved through the SimpleControllerIO interface.
- Retrieving the absolute position and orientation of links (rigid bodies) is also permitted
  (there is no need for position/posture estimation).

#### Simulator setting
- The AISTsimulator physics engine for simulation.
  Switching to other physics engines. or changing the parameters of the AISTsimulator is not permitted.
- A sample .cnoid project file is provided for each challenge theme.
  Use the simulator setting described in this file for simulation.

#### Operating the robot
- The robot can be operated with an input device, typically a joystick.
- The operator can operate the robot while monitoring the robot on the 3D view of Choreonoid.


## Quick Start
- See [here](https://ytazz.github.io/vnoid/build_sample_2022.html) for how to build and run the sample.

## Committee

- HVAC2023 is hosted by the Special Interest Group on Humanoid Robotics of the Robotics Society of Japan (RSJ-SIG-HR).
- HVAC2023 is run by volunteers consisting of members and non-members of RSJ-SIG-HR.
  - Commitee members
    - Yuichi Tazaki
    - Takumi Kamioka
    - Tomomichi Sugihara
    - Masaki Murooka
    - Kenji Hashimoto
    - Ikuo Mizuuchi
    - Yohei Kakiuchi
    - Shin'ichiro Nakaoka


## Asking Questions
- Post your question at [https://github.com/ytazz/vnoid/discussions](https://github.com/ytazz/vnoid/discussions)

## Old pages
- [Challenge 2021 page](https://ytazz.github.io/vnoid/index2021.html)
- [Challenge 2022 page](https://ytazz.github.io/vnoid/index2022.html)