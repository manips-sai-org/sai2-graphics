set(EXAMPLE_NAME 02-update_rendering)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_LIBRARIES}
)

# export resources such as model files.
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf rbot.urdf DESTINATION ${RESOURCE_DIR})
