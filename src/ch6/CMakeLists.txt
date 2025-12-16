aux_source_directory(. SRC_LIST)
add_executable(ch6 ${SRC_LIST})
target_link_libraries(ch6 ${CERES_LIBRARIES} common)