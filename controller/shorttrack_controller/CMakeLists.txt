include_directories("../../src")

add_cnoid_simple_controller(vnoid_shorttrack_controller  main.cpp)

target_link_libraries(vnoid_shorttrack_controller vnoid_lib)
