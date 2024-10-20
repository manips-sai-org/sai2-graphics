set(EXAMPLE_NAME 99-robot_graphics)

# - SAI2-COMMON
FIND_PACKAGE(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
	${SAI2-COMMON_LIBRARIES}
)

# export resources such as model files.
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf anymal.urdf HRP4c.urdf HRP4c_modified.urdf panda_arm.urdf DESTINATION ${RESOURCE_DIR})
