set(VNOID_HEADERS
	"bvh.h"
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
    "types.h"
	)
set(VNOID_SOURCES
	"bvh.cpp"
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
	)
	
if(VNOID_BUILD_CNOID)
    add_compile_definitions(VNOID_BUILD_CNOID)
	set(VNOID_HEADERS ${VNOID_HEADERS} "robot.h" "visualizer.h")
	set(VNOID_SOURCES ${VNOID_SOURCES} "robot.cpp" "visualizer.cpp")
endif()

if(VNOID_BUILD_MUJOCO)
    add_compile_definitions(VNOID_BUILD_MUJOCO)
    find_package(Eigen3)
    find_package(mujoco)

	set(VNOID_HEADERS ${VNOID_HEADERS} "robot_mujoco.h")
	set(VNOID_SOURCES ${VNOID_SOURCES} "robot_mujoco.cpp")
endif()

if(VNOID_BUILD_CNOID)
	choreonoid_add_library(vnoid_lib STATIC ${VNOID_HEADERS} ${VNOID_SOURCES})
	target_link_libraries(vnoid_lib CnoidBody)
	choreonoid_add_body_handler(SwingBridgeHandler SwingBridgeHandler.cpp)
else()
	add_library(vnoid_lib STATIC ${VNOID_HEADERS} ${VNOID_SOURCES})
	install(TARGETS vnoid_lib
		EXPORT        vnoidExport
		LIBRARY       DESTINATION lib
		INCLUDES      DESTINATION include
		PUBLIC_HEADER DESTINATION include
	)

	install(EXPORT vnoidExport
		FILE         vnoidConfig.cmake
		DESTINATION  cmake
	)
endif()

if(VNOID_BUILD_MUJOCO)
    target_link_libraries(vnoid_lib mujoco::mujoco Eigen3::Eigen)
endif()

