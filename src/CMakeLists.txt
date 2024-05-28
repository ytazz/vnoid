set(VNOID_BUILD_CNOID  true  CACHE BOOL "build vnoid to be used as Choreonoid simple controller")
set(VNOID_BUILD_MUJOCO false CACHE BOOL "build vnoid as mujoco simulation program")

set(VNOID_HEADERS
	"fksolver.h"
    "iksolver.h"
    "filter.h"
    "footstep.h"
    "footstep_planner.h"
    "ground_estimator.h"
	"robot_base.h"
	"rollpitchyaw.h"
	"stabilizer.h"
    "stepping_controller.h"
    "visualizer.h"
	)
set(VNOID_SOURCES
    "fksolver.cpp"
    "iksolver.cpp"
    "filter.cpp"
    "footstep.cpp"
    "footstep_planner.cpp"
    "ground_estimator.cpp"
	"robot_base.cpp"
	"rollpitchyaw.cpp"
	"stabilizer.cpp"
    "stepping_controller.cpp"
    "visualizer.cpp"
	)
	
if(VNOID_BUILD_CNOID)
    add_compile_definitions(VNOID_BUILD_CNOID)
	set(VNOID_HEADERS ${VNOID_HEADERS} "robot.h")
	set(VNOID_SOURCES ${VNOID_SOURCES} "robot.cpp")
endif()

if(VNOID_BUILD_MUJOCO)
    add_compile_definitions(VNOID_BUILD_MUJOCO)
    find_package(mujoco)

	set(VNOID_HEADERS ${VNOID_HEADERS} "robot_mujoco.h")
	set(VNOID_SOURCES ${VNOID_SOURCES} "robot_mujoco.cpp")
endif()

choreonoid_add_library(vnoid_lib STATIC ${VNOID_HEADERS} ${VNOID_SOURCES})

target_link_libraries(vnoid_lib CnoidBody)

if(VNOID_BUILD_MUJOCO)
    target_link_libraries(vnoid_lib mujoco::mujoco)
endif()

choreonoid_add_body_handler(SwingBridgeHandler SwingBridgeHandler.cpp)

