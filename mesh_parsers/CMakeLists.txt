#project(mesh_parsers)
file(GLOB mesh_parsers_src
    	"*.h"
    	"*.cpp"
)

add_library(mesh_parsers
	${mesh_parsers_src}
)

#find_package(CGAL QUIET)
#target_link_libraries(mesh_parsers graphics mesh_utils ${CGAL_LIBRARIES})
target_include_directories(mesh_parsers PUBLIC .)
