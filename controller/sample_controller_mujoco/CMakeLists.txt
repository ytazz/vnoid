include_directories("../../src")

find_package(mujoco)
find_package(glfw3)

#add_cnoid_simple_controller(vnoid_sample_controller  main.cpp myrobot.cpp myrobot.h)
add_executable(vnoid_sample_controller_mujoco
 main.cpp
 myrobot.cpp
 myrobot.h
)

target_link_libraries(vnoid_sample_controller_mujoco
 vnoid_lib
 mujoco::mujoco
 glfw
)

install(TARGETS vnoid_sample_controller_mujoco
  RUNTIME       DESTINATION bin
  )
