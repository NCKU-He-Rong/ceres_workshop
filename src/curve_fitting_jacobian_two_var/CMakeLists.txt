include_directories(include)
aux_source_directory(. SRC_LIST)
add_executable(curve_fitting_jacobian_two_var ${SRC_LIST})
target_link_libraries(curve_fitting_jacobian_two_var ${CERES_LIBRARIES} fileio)