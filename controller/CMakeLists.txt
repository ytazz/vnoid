if(VNOID_BUILD_CNOID)
	add_subdirectory(sample_controller)
	add_subdirectory(shorttrack_controller)

	add_subdirectory(demos)
	add_subdirectory(tutorials)
endif()

if(VNOID_BUILD_MUJOCO)
	add_subdirectory(sample_controller_mujoco)
endif()

