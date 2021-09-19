find_package(Springhead)
find_package(Scenebuilder)
find_package(Capt)
find_package(GLEW)

include_directories("../../src")

add_cnoid_simple_controller(BipedAvoid  main.cpp myrobot.cpp myrobot.h)

target_link_libraries(BipedAvoid BipedCnoid)
target_link_libraries(BipedAvoid Springhead::Springhead)
target_link_libraries(BipedAvoid Scenebuilder)
target_link_libraries(BipedAvoid capt_lib)
