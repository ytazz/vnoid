set(VNOID_BUILD_CNOID  true  CACHE BOOL "build vnoid to be used as Choreonoid simple controller")
set(VNOID_BUILD_MUJOCO false CACHE BOOL "build vnoid as mujoco simulation program")

if(VNOID_BUILD_CNOID)
	install(DIRECTORY model   DESTINATION ${CNOID_SHARE_SUBDIR})
	install(DIRECTORY project DESTINATION ${CNOID_SHARE_SUBDIR})
else()
	project(vnoid)
	install(DIRECTORY model   DESTINATION share)
endif()

add_subdirectory(src)
add_subdirectory(controller)

if(VNOID_BUILD_CNOID)
	add_subdirectory(plugin)
endif()
