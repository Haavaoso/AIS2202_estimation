find_package(Eigen3 CONFIG REQUIRED)


# Create the executable
add_executable(estimation kalman_filter.cpp)

# Link Eigen3 libraries
target_link_libraries(estimation PUBLIC Eigen3::Eigen)

# Include rapidcsv headers
target_include_directories(estimation PUBLIC ${RAPIDCSV_INCLUDE_DIRS} include)


