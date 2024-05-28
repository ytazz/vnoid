if(VNOID_BUILD_CNOID)
	add_subdirectory(sample_controller)
	add_subdirectory(shorttrack_controller)

	add_subdirectory(demos)
endif()

if(VNOID_BUILD_MUJOCO)
	add_subdirectory(sample_controller_mujoco)
endif()

