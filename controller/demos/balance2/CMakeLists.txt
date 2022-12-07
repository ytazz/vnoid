include_directories("../../../src")

add_cnoid_simple_controller(vnoid_demos_balance2_controller
  ../src/robot_demo.h
  ../src/robot_demo.cpp
  main.cpp
  myrobot.cpp
  myrobot.h)

target_link_libraries(vnoid_demos_balance2_controller vnoid_lib)
