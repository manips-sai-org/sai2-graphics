# include CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# include GLFW3
FIND_PACKAGE(glfw3 QUIET)

# - SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-GRAPHICS_INCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (04-change_worlds_at_runtime main.cpp)

# and link the library against the executable
FIND_LIBRARY(GLFW_LIB glfw)
TARGET_LINK_LIBRARIES (04-change_worlds_at_runtime
	${SAI2-GRAPHICS_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${CHAI3D_LIBRARIES}
	${GLFW_LIB})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf world2.urdf rbot.urdf DESTINATION ${RESOURCE_DIR})
