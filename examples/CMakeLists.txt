function(add_example folder_name target_name source_file)
  add_executable(${target_name})
  set_target_properties(${target_name} PROPERTIES FOLDER ${folder_name})

  target_sources(${target_name}
  PRIVATE
    ${source_file}
  )

  target_link_libraries(${target_name} PRIVATE pcp::pcp)

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