set(EXAMPLE_NAME 08-get_rgb_data_multiworld)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)
ADD_EXECUTABLE (test_headless test.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
)
TARGET_LINK_LIBRARIES (test_headless
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
)

# export resources such as model files.
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf sphbot.urdf DESTINATION ${RESOURCE_DIR})
