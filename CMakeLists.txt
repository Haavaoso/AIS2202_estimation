cmake_minimum_required(VERSION 3.28.1)
project(AIS4104-assignments)

set(CMAKE_CXX_STANDARD 20)


find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include(FetchContent)
FetchContent_Declare(
        matplotlibcpp
        GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
        1121c8a1328324923f36ab0df9537e2849f41a87
)
FetchContent_MakeAvailable(matplotlibcpp)

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

add_subdirectory(estimation)
add_subdirectory(parameter_estimation)

