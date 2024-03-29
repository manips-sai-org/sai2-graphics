set(EXAMPLE_NAME 05-apply_ui_force)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-GRAPHICS_EXAMPLES_LIBRARIES}
)