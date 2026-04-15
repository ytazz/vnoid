include_directories("../../../src")

add_cnoid_simple_controller(vnoid_tutorial1_controller
  main.cpp
  myrobot.cpp
  myrobot.h)

target_link_libraries(vnoid_tutorial1_controller vnoid_lib)
