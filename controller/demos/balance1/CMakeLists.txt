include_directories("../../../src")

add_cnoid_simple_controller(vnoid_demos_balance1_controller  main.cpp myrobot.cpp myrobot.h)

target_link_libraries(vnoid_demos_balance1_controller vnoid_lib)
