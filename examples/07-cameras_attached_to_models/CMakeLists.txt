set(EXAMPLE_NAME 07-cameras_attached_to_models)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
)