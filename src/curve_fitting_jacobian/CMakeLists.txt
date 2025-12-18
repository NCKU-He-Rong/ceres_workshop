aux_source_directory(. SRC_LIST)
add_executable(curve_fitting_jacobian ${SRC_LIST})
target_link_libraries(curve_fitting_jacobian ${CERES_LIBRARIES} fileio)