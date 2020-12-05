include(FetchContent)
set(LIBIGL_USE_STATIC_LIBRARY     ON  CACHE INTERNAL "Use libigl as static library" )
set(LIBIGL_WITH_CGAL              OFF CACHE INTERNAL "Use CGAL"                     )
set(LIBIGL_WITH_COMISO            OFF CACHE INTERNAL "Use CoMiso"                   )
set(LIBIGL_WITH_CORK              OFF CACHE INTERNAL "Use Cork"                     )
set(LIBIGL_WITH_EMBREE            OFF CACHE INTERNAL "Use Embree"                   )
set(LIBIGL_WITH_MATLAB            OFF CACHE INTERNAL "Use Matlab"                   )
set(LIBIGL_WITH_MOSEK             OFF CACHE INTERNAL "Use MOSEK"                    )
set(LIBIGL_WITH_PNG               OFF CACHE INTERNAL "Use PNG"                      )
set(LIBIGL_WITH_TETGEN            OFF CACHE INTERNAL "Use Tetgen"                   )
set(LIBIGL_WITH_TRIANGLE          OFF CACHE INTERNAL "Use Triangle"                 )
set(LIBIGL_WITH_PREDICATES        OFF CACHE INTERNAL "Use exact predicates"         )
set(LIBIGL_WITH_XML               OFF CACHE INTERNAL "Use XML"                      )
set(LIBIGL_WITHOUT_COPYLEFT       OFF CACHE INTERNAL "Disable Copyleft libraries"   )
set(LIBIGL_WITH_OPENGL            ON  CACHE INTERNAL "download libigl with opengl"  )
set(LIBIGL_WITH_OPENGL_GLFW       ON  CACHE INTERNAL "download libigl with glfw"    )
set(LIBIGL_WITH_OPENGL_GLFW_IMGUI ON  CACHE INTERNAL "download libigl with imgui"   )
FetchContent_Declare(
  _libigl
  GIT_REPOSITORY https://github.com/libigl/libigl
  GIT_TAG        v2.2.0
)
FetchContent_MakeAvailable(_libigl)

function(add_example folder_name target_name source_file)
  add_executable(${target_name})
  set_target_properties(${target_name} PROPERTIES FOLDER ${folder_name})

  target_sources(${target_name}
  PRIVATE
    ${source_file}
  )

  target_link_libraries(${target_name} PRIVATE pcp::pcp igl::core igl::opengl_glfw_imgui)

  include(GNUInstallDirs)

  install(
    TARGETS ${target_name}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endfunction()

add_example("examples" pcp-example-normals-estimation normals_estimation.cpp)
add_example("examples" pcp-example-ply ply.cpp)
add_example("examples" 
  pcp-example-tangent-plane-surface-reconstruction 
  tangent_plane_surface_reconstruction.cpp)
add_example("examples" 
  pcp-example-add-noise-to-point-cloud 
  add_noise_to_point_cloud.cpp)
add_example("examples" 
  pcp-example-filter-point-cloud-by-density 
  filter_point_cloud_noise_by_density.cpp)