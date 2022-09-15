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

Each function is implemented by about 100 lines of code,
 so it will serve as a good learning material for beginners.

### Building and running vnoid

- Using your git client, clone Choreonoid from [here](https://github.com/choreonoid/choreonoid), and clone vnoid from [here](https://github.com/ytazz/vnoid)
  as a submodule of Choreonoid (so the directories will look like choreonoid/ext/vnoid).
- Configure Chorenoid on Cmake. 
  If you have cloned vnoid below a directory other than choreonoid/ext/, then
  add the path to the vnoid directory to ADDITIONAL_EXT_DIRECTORIES. 
  If you have cloned it to chorenoid/ext/, no need to do this.
  Configure, generate, build, and install.
  As a result, Choreonoid will be installed to the location specified by CMAKE_INSTALL_PREFIX.
- Run Choreonoid, load a project file [install directory of Choreonoid]/share/project/[project name].cnoid,
  where [project name] is one of the following for the 2022 challenge.
	- vnoid_sample_project_2022_athletics (for athletics)
	- vnoid_sample_project_2022_shorttrack (for short track)
	- vnoid_sample_project_2022_performance (for free-style performance)
- Run simulation.
- Stop simulation.
- Save log data.
	- Right click "WorldLogFile", select "Save project as log playback archive", and save.
	- There will be a log file named "[logname].cnoid", and a folder with the same name including all log data.
	- Archive them into a single zip file, and submit it.

### Creating your own robot

If you are not familiar with creating a robot model and its controller, the easiest way would be to modify your local copy of vnoid.

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

Copy vnoid/project/vnoid_sample_project_2022_****.cnoid to my_project.cnoid in the same directory.
Open my_project.cnoid.
Find the reference to sample_robot.body and replace it with my_robot.body.
Similarly, find the reference to vnoid_sample_controller.dll and replace it with my_controller.dll.

When all the above procedure is finished, build and install Choreonoid.
You will find your project file in [Install directory of Choreonoid]/share/project/.
Open it on Choreonoid, and run simulation.

