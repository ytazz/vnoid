# Setting up vnoid

vnoid consists of a core library and a collection of demo projects.
If you want to run demos, you need to build vnoid as a subproject of the cmake project tree of Choreohoid.

## Common instructions

- On the terminal, create and move to the directory where you want to clone Choreonoid.

- Using your git client, clone Choreonoid from [here](https://github.com/choreonoid/choreonoid).
```
git clone https://github.com/choreonoid/choreonoid
```

- Move to choreonoid/ext/.
  Clone vnoid from [here](https://github.com/ytazz/vnoid)
```
cd choreonoid/ext
git clone https://github.com/ytazz/vnoid
```
  Note that you can clone vnoid into another location below choreonoid directory, for example, choreonoid/vnoid.
  In this case, you have to specify the location of vnoid source tree by the cmake variable ADDITIONAL_EXT_DIRECTORIES.

- Configure Chorenoid on Cmake. 
  Follow general instructions provided in Choreonoid web page [here](https://choreonoid.org/en/manuals/latest/install/install.html).
  See platform-specific and vnoid-specific notes on configuration below.

- Build the generated project on your build platform.
  As a result, Choreonoid will be installed to the location specified by CMAKE_INSTALL_PREFIX.
  
- Run Choreonoid, load [install directory of Choreonoid]/share/project/vnoid_sample_project.cnoid

- Run simulation.

### Configuration on Windows
  On Windows environment, enabling USE_SUBSYSTEM_CONSOLE option is convenient for printf-based debugging.
  Be careful of setting CMAKE_INSTALL_PREFIX properly (see [here](https://choreonoid.org/en/manuals/latest/install/build-windows.html)).

### vnoid configuration options

  VNOID_BUILD_CNOID option is used to build sample controllers that can be loaded from Choreonoid projects.
  VNOID_BUILD_MUJOCO option is used to build mujoco samples.

  If BUILD_VNOID_VISUALIZER_PLUGIN option is set, the visualization function of vnoid is built as a Choreonoid plugin.
  
### Setup on Linux

### Building vnoid with mujoco
  You need mujoco and glfw3 properly installed in your environment.
  On cmake, enable VNOID_BUILD_MUJOCO option and configure.
  You may need to set CMAKE_PREFIX_PATH properly to let cmake locate the related packages.
  
  After build and install, the executables of mujoco samples (currently there is only one) will be
  copied to the location specified by CMAKE_INSTALL_PREFIX.
  On Windows, if you built mujoco as a dll, you also need to manually copy mujoco.dll to the same location.
