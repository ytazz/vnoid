
set(VNOID_HEADERS
    "footstep.h"
    "iksolver.h"
	"robot.h"
	)
set(VNOID_SOURCES
    "footstep.cpp"
    "iksolver.cpp"
	"robot.cpp"
	)

add_cnoid_library(vnoid
 ${VNOID_HEADERS}
 ${VNOID_SOURCES}
 )

target_link_libraries(vnoid CnoidBody)

