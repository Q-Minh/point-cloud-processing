add_executable(pcp-tests)
set_target_properties(pcp-tests PROPERTIES FOLDER pcp-tests)

target_sources(pcp-tests
PRIVATE
  "main.cpp"
  "algorithm/average_distance_to_neighbors.cpp"
  "algorithm/bilateral_filter.cpp"
  "algorithm/estimate_normals.cpp" 
  "algorithm/estimate_tangent_planes.cpp"
  "algorithm/hierarchy_simplification.cpp"
  "algorithm/random_simplification.cpp"
  "algorithm/surface_nets.cpp"
  "algorithm/wlop.cpp"
  "common/aabb.cpp"
  "common/plane3d.cpp"
  "common/tokenize.cpp"
  "common/normal_estimation.cpp"
  "graph/undirected_knn_adjacency_list.cpp"
  "graph/directed_adjacency_list.cpp" 
  "graph/minimum_spanning_tree.cpp"
  "graph/search.cpp"
  "io/custom_point.hpp"
  "io/obj.cpp"
  "io/ply.cpp"
  "kdtree/iterator.cpp"
  "kdtree/kdtree_insertion.cpp"
  "kdtree/kdtree_range_search.cpp"
  "kdtree/knn.cpp"
  "octree/octree_deletion.cpp"
  "octree/octree_find.cpp"
  "octree/octree_insertion.cpp"
  "octree/octree_iterator.cpp"
  "octree/octree_knn.cpp"
  "octree/octree_range_search.cpp"
  "type/property_map.cpp")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    option(PCP_COVERAGE "Enable code coverage reports for gcc/clang" FALSE)
    if (PCP_COVERAGE)
        target_compile_options(pcp-tests 
          PRIVATE 
            $<$<CONFIG:Debug>:--coverage -O0 -ftest-coverage>
        )
        target_link_options(pcp-tests 
          PRIVATE 
            $<$<CONFIG:Debug>:--coverage>
        )
    endif()
endif()

target_compile_options(pcp-tests
PRIVATE
  $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:
    -Wall
    -Wextra
    -Wpedantic
    -Wdouble-promotion
    -Wnull-dereference
    -Wimplicit-fallthrough
    -Wswitch-default
    -Wswitch-enum
    -Wunused-const-variable
    -Wuninitialized
    -Wduplicate-decl-specifier
    -Wduplicate-enum
    -Wduplicate-method-arg
    -Wduplicate-method-match
    -Wfloat-equal
    -Wshadow-all
    -Wcast-qual
    -Wcast-align
    -Wconversion
    -Wdangling-else
    -Wsign-conversion
    -Wfloat-conversion
    -fstrict-enums
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wzero-as-null-pointer-constant
    -Wno-unused-local-typedef
  >
  $<$<COMPILE_LANG_AND_ID:CXX,GNU>:
    -Wall
    -Wextra
    -Wpedantic
    -Wdouble-promotion
    -Wnull-dereference
    -Wimplicit-fallthrough
    -Wswitch-default
    -Wswitch-enum
    -Wunused-const-variable=2
    -Wuninitialized
    -Walloc-zero
    -Wduplicated-branches
    -Wduplicated-cond
    -Wfloat-equal
    -Wshadow=local
    -Wcast-qual
    -Wcast-align
    -Wconversion
    -Wdangling-else
    -Wsign-conversion
    -Wfloat-conversion
    -Wlogical-op
    -fstrict-enums
    -Wnoexcept
    -Wnon-virtual-dtor
    -Wstrict-null-sentinel
    -Wold-style-cast
    -Woverloaded-virtual
    -Wmisleading-indentation
    -Wzero-as-null-pointer-constant
    -Wplacement-new=2
    -Wsuggest-final-types
    -Wsuggest-final-methods
    -Wsuggest-override
    -Wno-unused-local-typedef
  >
  $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:
    /permissive-
    /W4
    /w44242
    /w44263
    /w44265
    /w44287
    /w44289
    /w44296
    /w44311
    /w44545
    /w44546
    /w44547
    /w44549
    /w44555
    /w44619
    /w44640
    /w44826
    /w44928
  >
)

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.2
)
FetchContent_MakeAvailable(Catch2)

target_link_libraries(pcp-tests PRIVATE pcp::pcp Catch2::Catch2)

get_target_property(catch2_include_directories Catch2::Catch2 INTERFACE_INCLUDE_DIRECTORIES)
include_directories(SYSTEM ${catch2_include_directories})

include(GNUInstallDirs)

install(
  TARGETS pcp-tests
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
