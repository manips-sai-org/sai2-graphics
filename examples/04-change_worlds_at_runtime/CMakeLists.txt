set(EXAMPLE_NAME 04-change_worlds_at_runtime)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_LIBRARIES}
)

# export resources such as model files.
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf world2.urdf rbot.urdf DESTINATION ${RESOURCE_DIR})
