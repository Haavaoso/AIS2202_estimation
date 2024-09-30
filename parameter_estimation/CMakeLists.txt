find_package(Eigen3 CONFIG REQUIRED)

# Find the rapidcsv header file path
find_path(RAPIDCSV_INCLUDE_DIRS
        NAMES rapidcsv.h
        PATHS /usr/include /usr/local/include
        REQUIRED)

# Check if rapidcsv was found
if(NOT RAPIDCSV_INCLUDE_DIRS)
    message(FATAL_ERROR "rapidcsv.h not found.")
endif()

# Create the executable
add_executable(parameter_estimation main.cpp)

# Link Eigen3 libraries
target_link_libraries(parameter_estimation PUBLIC Eigen3::Eigen)

file(COPY "${PROJECT_SOURCE_DIR}/dataset" DESTINATION "${CMAKE_BINARY_DIR}")

include_directories("${PROJECT_SOURCE_DIR}/dataset")

# Include rapidcsv headers
target_include_directories(parameter_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})


