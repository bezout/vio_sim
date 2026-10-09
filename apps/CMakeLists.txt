set(viosim_app_source_dir "${CMAKE_CURRENT_LIST_DIR}")

add_executable(viosim "${viosim_app_source_dir}/main.cpp")
target_link_libraries(viosim PRIVATE viosim_core)

if(BUILD_TESTING)
  # Explicit hardware test; do not require an EGL GPU in the regular CTest suite.
  add_executable(test_viosim_gpu_rays "${viosim_app_source_dir}/test_gpu_rays.cpp")
  target_link_libraries(test_viosim_gpu_rays PRIVATE viosim_core)
endif()

add_executable(viosim_lab_preview "${viosim_app_source_dir}/lab_preview.cpp")
target_link_libraries(viosim_lab_preview PRIVATE viosim_core)

# Keep the executable self-contained when it is run from the build tree.
add_custom_command(TARGET viosim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${viosim_app_source_dir}/../assets
    $<TARGET_FILE_DIR:viosim>/assets)

install(TARGETS viosim RUNTIME DESTINATION bin)
install(TARGETS viosim_lab_preview RUNTIME DESTINATION bin)
