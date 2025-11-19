# Setting up vnoid

vnoid consists of a core library and a collection of demo projects.
If you want to run demos, you need to build vnoid as a subproject of the cmake project tree of Choreohoid.

## Setup on Windows

- Using your git client, clone Choreonoid from [here](https://github.com/choreonoid/choreonoid).
  Normally you can do it by
```
git clone https://github.com/choreonoid/choreonoid
```

 and clone vnoid from [here](https://github.com/ytazz/vnoid)
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

## Setup on Linux

## Building vnoid library only


