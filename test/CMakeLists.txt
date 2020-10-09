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

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.2
)
FetchContent_MakeAvailable(Catch2)

target_link_libraries(tests PRIVATE pcp::pcp Catch2::Catch2)

include(GNUInstallDirs)

install(
  TARGETS tests
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)