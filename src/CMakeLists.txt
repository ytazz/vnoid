
set(VNOID_HEADERS
    "footstep.h"
    "iksolver.h"
	"robot.h"
	"rollpitchyaw.h"
	)
set(VNOID_SOURCES
    "footstep.cpp"
    "iksolver.cpp"
	"robot.cpp"
	"rollpitchyaw.cpp"
	)

add_cnoid_library(vnoid_lib
 ${VNOID_HEADERS}
 ${VNOID_SOURCES}
 )

target_link_libraries(vnoid_lib CnoidBody)

