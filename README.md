# vnoid
vnoid is a simple library used in samples codes of Humanoid Virtual Athletics Challenge.
It provides basic functionality of humanoid robot including:
- SimpleController interface with Choreonoid
- Foot step planning
- Inverse kinematics
- Stepping control
- Stabilization control

The vnoid library is intended to be as simple as possible in order to serve as a good entry point of learning for beginners.
Each function is implemented by about 100 lines of code.

vnoid is primarily designed to implement a simple robot controller that runs on Choreonoid.
However, one can also build a stand-along mujoco simulation program using vnoid.

## How to use vnoid with Choreonoid

In order to implement your own controller for Choreonoid, you need to be able to build Chorenoid from source.
See Chorenoid Manual ([jp](https://choreonoid.org/ja/documents/latest/index.html), [en](https://choreonoid.org/en/documents/latest/index.html)) for detailed instruction.

### Building and running vnoid

- Using your git client, clone Choreonoid from [here](https://github.com/choreonoid/choreonoid), and clone vnoid from [here](https://github.com/ytazz/vnoid)
  as a submodule of Choreonoid (so the directories will look like choreonoid/ext/vnoid).
- Configure Chorenoid on Cmake. 
  If you have cloned vnoid below a directory other than choreonoid/ext/, then
  add the path to the vnoid directory to ADDITIONAL_EXT_DIRECTORIES. 
  If you have cloned it to chorenoid/ext/, no need to do this.
  On Windows environment, enabling USE_SUBSYSTEM_CONSOLE option is convenient for printf-based debugging.
  Configure, generate, build, and install.
  As a result, Choreonoid will be installed to the location specified by CMAKE_INSTALL_PREFIX.
- Run Choreonoid, load [install directory of Choreonoid]/share/project/vnoid_sample_project.cnoid
- Run simulation.

### Creating your own robot

If you are not familiar with creating a robot model and its controller, the easiest way would be
 to modify your local copy of vnoid.

#### Creating your robot model

Copy the directory vnoid/model/sample_robot to vnoid/model/my_robot (my_robot part is arbitrary).
Inside vnoid/model/my_robot, rename sample_robot.body to my_robot.body.
my_robot.body is the description file of your robot model.
Edit it as you like.
You could also replace the 3D models used for visualization and collision detection.

#### Creating your controller

Copy the directory vnoid/controller/sample_controller to vnoid/controller/my_controller (again, my_controller part is arbitrary).
Inside vnoid/controller/my_controller, edit CMakeLists.txt and replace vnoid_sample_controller to my_controller.
Edit main.cpp and possibly other files to implement your own controller.

Also edit CMakeLists.txt right below vnoid/controller, and add add_subdirectory(my_controller).

#### Creating your project

Copy vnoid/project/vnoid_sample_project.cnoid to my_project.cnoid in the same directory.
Open my_project.cnoid.
Find the reference to sample_robot.body and replace it with my_robot.body.
Similarly, find the reference to vnoid_sample_controller.dll and replace it with my_controller.dll.

When all the above procedure is finished, build and install Choreonoid.
You will find your project file in [Install directory of Choreonoid]/share/project/.
Open it on Choreonoid, and run simulation.

## How to use vnoid with mujoco

Install [mujoco](https://github.com/google-deepmind/mujoco).

### Building and running vnoid

- Using your git client, clone vnoid from [here](https://github.com/ytazz/vnoid).
- Run Cmake. 
  Check VNOID_BUILD_MUJOCO. Uncheck VNOID_BUILD_CNOID, which is checked by default.
  Specify installation path in CMAKE_INSTALL_PREFIX.
  Adding ".../vnoid" to the installation path is recommended to avoid having installed files mixed with other stuff.
  Configure, generate, build, and install.
- Run the executable in the bin directory of the install path.
