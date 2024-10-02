cmake_minimum_required(VERSION 3.24.2)
project(AIS4104-assignments)

set(CMAKE_CXX_STANDARD 20)


file(COPY "${CMAKE_SOURCE_DIR}/dataset" DESTINATION "${CMAKE_BINARY_DIR}")

add_subdirectory(estimation)
add_subdirectory(parameter_estimation)
add_subdirectory(state_estimation)

add_dependencies(state_estimation estimation)