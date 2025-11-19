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

  On Windows environment, enabling USE_SUBSYSTEM_CONSOLE option is convenient for printf-based debugging.
  As a result, Choreonoid will be installed to the location specified by CMAKE_INSTALL_PREFIX.
  
-- vnoid configuration options


- Run Choreonoid, load [install directory of Choreonoid]/share/project/vnoid_sample_project.cnoid
- Run simulation.

## Setup on Linux

## Building vnoid library only


