
set(VNOID_HEADERS
    "iksolver.h"
    "filter.h"
    "footstep.h"
    "footstep_planner.h"
	"robot.h"
	"rollpitchyaw.h"
	"stabilizer.h"
    "stepping_controller.h"
	)
set(VNOID_SOURCES
    "iksolver.cpp"
    "filter.cpp"
    "footstep.cpp"
    "footstep_planner.cpp"
	"robot.cpp"
	"rollpitchyaw.cpp"
	"stabilizer.cpp"
    "stepping_controller.cpp"
	)

choreonoid_add_library(vnoid_lib SHARED ${VNOID_HEADERS} ${VNOID_SOURCES})

target_link_libraries(vnoid_lib CnoidBody)

