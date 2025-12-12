include_directories(include)
add_executable(manual_jac_muti_dim main.cpp)
target_link_libraries(manual_jac_muti_dim ${CERES_LIBRARIES})