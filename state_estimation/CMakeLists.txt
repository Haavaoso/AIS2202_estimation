find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(state_estimation main.cpp)

add_compile_definitions(PROJECT_PATH="${CMAKE_SOURCE_DIR}/")

target_link_libraries(state_estimation PUBLIC Eigen3::Eigen estimation)
target_include_directories(state_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})
