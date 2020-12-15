add_executable(pcp-benchmarks)
set_target_properties(pcp-benchmarks PROPERTIES FOLDER pcp-benchmarks)
target_compile_options(pcp-benchmarks
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

target_sources(pcp-benchmarks
PRIVATE
  octree_benchmark.cpp
)

# do not include google benchmark tests in our project
set(BENCHMARK_ENABLE_GTEST_TESTS OFF)
set(BENCHMARK_ENABLE_TESTING OFF)
set(BENCHMARK_ENABLE_ASSEMBLY_TESTS OFF)
include(FetchContent)
FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.5.2
)
FetchContent_MakeAvailable(benchmark)

target_link_libraries(pcp-benchmarks PRIVATE pcp::pcp benchmark::benchmark benchmark::benchmark_main)

include(GNUInstallDirs)

install(
  TARGETS pcp-benchmarks
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
