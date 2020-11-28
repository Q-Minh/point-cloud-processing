cmake_minimum_required(VERSION 3.14)
project(pcp VERSION 0.0.1 LANGUAGES CXX)

option(PCP_BUILD_TESTS "build tests (requires catch2)" ON)
option(PCP_BUILD_BENCHMARKS "build benchmarks (requires google benchmark)" ON)
option(PCP_BUILD_EXAMPLES "build examples" ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(FetchContent)
FetchContent_Declare(
  eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen
  GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(eigen3)

add_library(pcp INTERFACE)
add_library(pcp::pcp ALIAS pcp)
target_compile_features(pcp INTERFACE cxx_std_17)
target_link_libraries(pcp INTERFACE eigen)
target_sources(pcp
INTERFACE
    $<BUILD_INTERFACE:

        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/pcp.hpp
        
        # common
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/axis_aligned_bounding_box.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/intersections.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/norm.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/point_predicates.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/sphere.hpp

        # common/normals
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/normals/normal.hpp

        # common/points
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/points/point.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/points/point_view.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/common/points/vertex.hpp

        # graph
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/graph/directed_adjacency_list.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/graph/minimum_spanning_tree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/graph/undirected_knn_adjacency_list.hpp

        # io
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/io/endianness.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/io/obj.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/io/ply.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/io/tokenize.hpp
        
        # octree
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/octree/octree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/octree/octree_iterator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/octree/octree_node.hpp

        # traits
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/graph_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/graph_vertex_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/iterable_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/knn_search_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/point_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/normal_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/range_traits.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pcp/traits/vector3d_traits.hpp
    >
)

target_include_directories(pcp
INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

include(GNUInstallDirs)

install(
    TARGETS pcp
    EXPORT pcp_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT pcp_targets
    FILE pcpTargets.cmake
    NAMESPACE pcp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pcp
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/pcpConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/pcpConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/pcpConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pcp
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/pcpConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/pcpConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pcp
)

export(
    EXPORT pcp_targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/pcpTargets.cmake
    NAMESPACE pcp::
)

if (PCP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

if (PCP_BUILD_BENCHMARKS)
    add_subdirectory(benchmark)
endif()

if (PCP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
