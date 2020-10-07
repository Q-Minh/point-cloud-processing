find_package(Catch2 CONFIG REQUIRED)
add_executable(tests)
set_target_properties(tests PROPERTIES FOLDER tests)
target_sources(tests
PRIVATE
    main.cpp
    obj.cpp
    octree_deletion.cpp
    octree_find.cpp
    octree_insertion.cpp
    octree_iterator.cpp
    octree_knn.cpp
    octree_range_search.cpp
    ply.cpp
    tokenize.cpp
)

target_link_libraries(tests PRIVATE pcp::pcp Catch2::Catch2)

include(GNUInstallDirs)

install(
    TARGETS tests
    # EXPORT tests_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)