set(VNOID_HEADERS
	"fksolver.h"
    "iksolver.h"
    "filter.h"
    "footstep.h"
    "footstep_planner.h"
    "ground_estimator.h"
	"robot.h"
	"rollpitchyaw.h"
	"stabilizer.h"
    "stepping_controller.h"
    "visualizer.h"
	)
set(VNOID_SOURCES
    "fksolver.cpp"
    "iksolver.cpp"
    "filter.cpp"
    "footstep.cpp"
    "footstep_planner.cpp"
    "ground_estimator.cpp"
	"robot.cpp"
	"rollpitchyaw.cpp"
	"stabilizer.cpp"
    "stepping_controller.cpp"
    "visualizer.cpp"
	)

choreonoid_add_library(vnoid_lib STATIC ${VNOID_HEADERS} ${VNOID_SOURCES})

target_link_libraries(vnoid_lib CnoidBody)

choreonoid_add_body_handler(SwingBridgeHandler SwingBridgeHandler.cpp)

