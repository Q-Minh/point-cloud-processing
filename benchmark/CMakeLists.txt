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
    /w14242
    /w14263
    /w14265
    /w14287
    /we4289
    /w14296
    /w14311
    /w14545
    /w14546
    /w14547
    /w14549
    /w14555
    /w14619
    /w14640
    /w14826
    /w14928
  >
)

target_sources(pcp-benchmarks
PRIVATE
  benchmark.cpp
)

set(BENCHMARK_ENABLE_GTEST_TESTS OFF)
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
