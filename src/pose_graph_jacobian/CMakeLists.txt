aux_source_directory(. SRC_LIST)
list(APPEND SRC_LIST include/pose_manifold.cpp)
add_executable(pose_graph_jacobian ${SRC_LIST})
target_link_libraries(pose_graph_jacobian ${CERES_LIBRARIES})