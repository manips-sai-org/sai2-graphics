CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (SAI2-GRAPHICS)
SET (PROJECT_VERSION 0.1.0) #major version is zero for now since api will certainly change in future

SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")

# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
	# Set the default build, and also the possible values of build type for cmake-gui
	SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
	SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# include Chai3d
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})

# include Sai2-Model
FIND_PACKAGE(SAI2-MODEL REQUIRED)

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)

# include sai2-URDF
find_package(SAI2-URDF REQUIRED)

# include Parser
SET(PARSER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/parser)
SET(PARSER_SOURCE ${PROJECT_SOURCE_DIR}/src/parser/UrdfToSai2Graphics.cpp)

# include Graphics
SET(SAI2-GRAPHICS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
SET(GRAPHICS_SOURCE ${PROJECT_SOURCE_DIR}/src/Sai2Graphics.cpp)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-GRAPHICS_INCLUDE_DIRS}
	${SAI2-MODEL_INCLUDE_DIRS}
	${SAI2-URDF_INCLUDE_DIRS}
	${CHAI3D_INCLUDE_DIRS}
	${PARSER_INCLUDE_DIR}
	)

# Create the library
ADD_LIBRARY (sai2-graphics STATIC
	${GRAPHICS_SOURCE}
	${PARSER_SOURCE}
	)

SET (SAI2-GRAPHICS_LIBRARIES sai2-graphics 
	${SAI2-URDF_LIBRARIES}
	)

# 
# export package
# 

# export package for use from the build tree
EXPORT (TARGETS sai2-graphics FILE ${PROJECT_BINARY_DIR}/SAI2-GRAPHICSTargets.cmake)
EXPORT (PACKAGE SAI2-GRAPHICS)

# configure package
SET (CONF_INCLUDE_DIRS ${SAI2-GRAPHICS_INCLUDE_DIRS} ${CHAI3D_INCLUDE_DIRS})
SET (CONF_LIBRARIES ${SAI2-GRAPHICS_LIBRARIES})

# create package configuration and version files
CONFIGURE_FILE(SAI2-GRAPHICSConfig.cmake.in "${PROJECT_BINARY_DIR}/SAI2-GRAPHICSConfig.cmake" @ONLY)
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI2-GRAPHICSConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# add examples
# ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/examples)
