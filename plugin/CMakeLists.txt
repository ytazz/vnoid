set(BUILD_VNOID_VISUALIZER_PLUGIN false CACHE BOOL "build MarkerVisualizerPlugin")

if(BUILD_VNOID_VISUALIZER_PLUGIN)
    add_subdirectory(visualizer)
endif()
