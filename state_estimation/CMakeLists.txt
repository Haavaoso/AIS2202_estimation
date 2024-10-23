find_package(Eigen3 CONFIG REQUIRED)


# Create the executable
add_executable(state_estimation kalman_filter.cpp)

# Link Eigen3 libraries
target_link_libraries(state_estimation PUBLIC Eigen3::Eigen)
