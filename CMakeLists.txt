cmake_minimum_required(VERSION 3.14)
project(octree)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

target_sources(${PROJECT_NAME}
INTERFACE
    include/axis_aligned_bounding_box.hpp
    include/intersections.hpp
    include/octree.hpp
    include/octree_iterator.hpp
    include/octree_node.hpp
    include/point.hpp
    include/sphere.hpp
)

target_include_directories(${PROJECT_NAME}
INTERFACE
    include/
)

find_package(Catch2 CONFIG REQUIRED)
add_executable(tests)
set_target_properties(tests PROPERTIES FOLDER tests)
target_sources(tests
PRIVATE
    src/main.cpp
    src/octree_insertion.cpp
    src/octree_iterator.cpp
    src/octree_knn.cpp
    src/octree_range_search.cpp
)

target_link_libraries(tests PRIVATE ${PROJECT_NAME} Catch2::Catch2)

find_package(benchmark CONFIG REQUIRED)
add_executable(benchmarks)
set_target_properties(benchmarks PROPERTIES FOLDER benchmarks)
target_sources(benchmarks
PRIVATE
    src/benchmark.cpp
)
target_link_libraries(benchmarks PRIVATE ${PROJECT_NAME} benchmark::benchmark benchmark::benchmark_main)