#project(graphics)

file(GLOB graphics_src
    	"*.h"
    	"*.c"
	"*.cpp"
)


add_library(graphics
	${graphics_src}
)

## For GLEW
# GLEW is a required package, so report config failure if not found
find_package(GLEW REQUIRED)
find_package(CGAL REQUIRED)
include_directories( $ENV{EIGEN3_INCLUDE_DIR} )
target_link_libraries(graphics geom_utils GLEW::GLEW ${CGAL_LIBRARIES})

message(${GLEW_INCLUDE_DIRS})
message(${GLEW_LIBRARIES})

target_include_directories(graphics PUBLIC .)
target_include_directories(graphics PUBLIC ${GLEW_INCLUDE_DIRS})