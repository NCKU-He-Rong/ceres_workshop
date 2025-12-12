include_directories(include)
add_executable(manual_jac_one_dim main.cpp)
target_link_libraries(manual_jac_one_dim ${CERES_LIBRARIES})