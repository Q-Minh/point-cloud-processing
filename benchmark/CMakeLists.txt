add_executable(benchmarks)
set_target_properties(benchmarks PROPERTIES FOLDER benchmarks)
target_sources(benchmarks
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

target_link_libraries(benchmarks PRIVATE pcp::pcp benchmark::benchmark benchmark::benchmark_main)

include(GNUInstallDirs)

install(
    TARGETS benchmarks
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
