cmake_minimum_required(VERSION 3.28.1)
project(AIS4104-assignments)

set(CMAKE_CXX_STANDARD 20)
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

include_directories("${CMAKE_SOURCE_DIR}/include")

file(COPY "${PROJECT_SOURCE_DIR}/dataset" DESTINATION "${CMAKE_BINARY_DIR}")

add_executable(complete_estimation main.cpp)

target_link_libraries(complete_estimation
        state_estimation_lib
        parameter_estimation_lib
        Eigen3::Eigen )

target_include_directories(complete_estimation
        PRIVATE ${RAPIDCSV_INCLUDE_DIRS}
        "${CMAKE_SOURCE_DIR}/estimation")

add_subdirectory(state_estimation)
add_subdirectory(parameter_estimation)
add_subdirectory(estimation)
