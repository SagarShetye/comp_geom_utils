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
target_link_libraries(graphics PUBLIC GLEW::GLEW)

message(${GLEW_INCLUDE_DIRS})
message(${GLEW_LIBRARIES})

target_include_directories(graphics PUBLIC .)
target_include_directories(graphics PUBLIC ${GLEW_INCLUDE_DIRS})