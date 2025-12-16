aux_source_directory(. SRC_LIST)
add_executable(curve_fitting_autodiff ${SRC_LIST})
target_link_libraries(curve_fitting_autodiff ${CERES_LIBRARIES} fileio)