set(SAI2-GRAPHICS_EXAMPLES_LIBRARIES
    ${SAI2-GRAPHICS_LIBRARIES} ${SAI2-MODEL_LIBRARIES} ${SAI2-URDF_LIBRARIES}
    ${CHAI3D_LIBRARIES})

set(EXAMPLES_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DEXAMPLES_FOLDER="${EXAMPLES_FOLDER}")

add_subdirectory(01-parse_world_and_robot)
add_subdirectory(02-update_rendering)
add_subdirectory(03-multiple_cameras)
add_subdirectory(04-change_worlds_at_runtime)
add_subdirectory(05-apply_ui_force)
add_subdirectory(06-force_sensor_display)
add_subdirectory(07-cameras_attached_to_models)
