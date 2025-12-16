aux_source_directory(. SRC_LIST)
add_executable(curve_fitting_numdiff ${SRC_LIST})
target_link_libraries(curve_fitting_numdiff ${CERES_LIBRARIES} fileio)