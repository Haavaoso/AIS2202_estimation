

# Create the executable
add_executable(parameter_estimation main.cpp)

# Link Eigen3 libraries
target_link_libraries(parameter_estimation PUBLIC Eigen3::Eigen)


# Include rapidcsv headers
target_include_directories(parameter_estimation
        PRIVATE ${RAPIDCSV_INCLUDE_DIRS}
        "${CMAKE_SOURCE_DIR}/estimation")


