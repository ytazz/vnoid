## Instruction for building the sample code of vnoid

You need basic knowledge of git and CMake.

In order to implement your own controller for Choreonoid, 
 you need to be able to build Chorenoid from source.
See Chorenoid Manual ([jp](https://choreonoid.org/ja/documents/latest/index.html), [en](https://choreonoid.org/en/documents/latest/index.html))
 for detailed instruction.

### About vnoid

vnoid is a simple library that provides basic functionality of humanoid robot.
It includes:
- SimpleController interface with Choreonoid
- Foot step planning
- Inverse kinematics
- Stepping control
- Stabilization control

Each function is implemented by about 100 lines of code.

### Building and running vnoid

- Using your git client, clone Choreonoid from [here](https://github.com/choreonoid/choreonoid), and clone vnoid from [here](https://github.com/ytazz/vnoid)
  as a submodule of Choreonoid (so the directories will look like choreonoid/vnoid).
- Configure Chorenoid on Cmake. Add the path to vnoid to ADDITIONAL_EXT_DIRECTORIES. Configure, generate, build, and install.
  As a result, Choreonoid will be installed to the location specified by CMAKE_INSTALL_PREFIX.
- Run Choreonoid, load [install directory of Choreonoid]/share/project/vnoid_sample_project.cnoid
- Run simulation.

### Creating your own robot

If you are not familiar with creating a robot model and its controller, the easiest way would be
 to modify your local copy of vnoid.

#### Creating your robot model

#### Creating your controller

#### Creating your project

