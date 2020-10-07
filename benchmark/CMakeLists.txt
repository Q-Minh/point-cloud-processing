find_package(benchmark CONFIG REQUIRED)
add_executable(benchmarks)
set_target_properties(benchmarks PROPERTIES FOLDER benchmarks)
target_sources(benchmarks
PRIVATE
    benchmark.cpp
)
target_link_libraries(benchmarks PRIVATE pcp::pcp benchmark::benchmark benchmark::benchmark_main)

include(GNUInstallDirs)

install(
    TARGETS benchmarks
    # EXPORT benchmarks_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
