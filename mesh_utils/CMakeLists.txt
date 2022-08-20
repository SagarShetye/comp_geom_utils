#project(mesh_utils)
file(GLOB mesh_utils_src
    	"*.h"
    	"*.cpp"
)

add_library(mesh_utils
	${mesh_utils_src}
)

find_package(CGAL REQUIRED)
target_link_libraries(mesh_utils graphics mesh_parsers ${CGAL_LIBRARIES})

target_include_directories(mesh_utils PUBLIC .)
