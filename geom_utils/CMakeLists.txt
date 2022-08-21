#project(geom_utils)

file(GLOB graphics_src
    	"*.h"
    	"*.c"
	"*.cpp"
)


add_library(geom_utils
	${graphics_src}
)

find_package(CGAL REQUIRED)
target_link_libraries(geom_utils ${CGAL_LIBRARIES})


target_include_directories(geom_utils PUBLIC .)
