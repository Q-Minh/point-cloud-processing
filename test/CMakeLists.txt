add_executable(pcp-tests)
set_target_properties(pcp-tests PROPERTIES FOLDER pcp-tests)

target_sources(pcp-tests
PRIVATE
  "main.cpp"
  "custom_point.hpp"
  "obj.cpp"
  "octree_deletion.cpp"
  "octree_find.cpp"
  "octree_insertion.cpp"
  "octree_iterator.cpp"
  "octree_knn.cpp"
  "octree_range_search.cpp"
  "ply.cpp"
  "undirected_knn_adjacency_list.cpp"
  "plane3d.cpp"
  "tokenize.cpp"
  "directed_adjacency_list.cpp" 
  "minimum_spanning_tree.cpp"
  "normal_estimation.cpp"
  "graph/dumb_vertex.hpp"
  "graph/search.cpp"
)

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

include(GNUInstallDirs)

install(
  TARGETS pcp-tests
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
