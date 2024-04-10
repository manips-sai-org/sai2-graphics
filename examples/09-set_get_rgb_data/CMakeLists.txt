find_package(SAI2-COMMON)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

set(EXAMPLE_NAME 09-set_get_rgb_data)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)
ADD_EXECUTABLE (test_redis test.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
	${SAI2-COMMON_LIBRARIES}
)
TARGET_LINK_LIBRARIES (test_redis
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
	${SAI2-COMMON_LIBRARIES}
)

# export resources such as model files.
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf sphbot.urdf DESTINATION ${RESOURCE_DIR})
