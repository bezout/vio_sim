cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(VioSim VERSION 1.0.0 LANGUAGES CXX)
  include(CTest)
  set(VIOSIM_STANDALONE ON)
else()
  set(VIOSIM_STANDALONE OFF)
endif()

option(VIOSIM_BUILD_APPS "Build the dataset generator and preview applications" ${VIOSIM_STANDALONE})

set(VIOSIM_CORE_SOURCES
  src/benchmark.cpp
  src/camera.cpp
  src/config.cpp
  src/dataset.cpp
  src/gpu_renderer.cpp
  src/image.cpp
  src/lab_room.cpp
  src/renderer.cpp
  src/scene.cpp
  src/simulation.cpp
  src/trajectory.cpp
  src/truth.cpp)

option(VIOSIM_OPENMP "Enable parallel VioSim frame rendering" ON)
option(VIOSIM_GPU "Enable the optional EGL/OpenGL compute renderer" ON)

add_subdirectory(long_motion)

find_package(PNG REQUIRED)
if(VIOSIM_OPENMP)
  find_package(OpenMP QUIET COMPONENTS CXX)
endif()

add_library(viosim_core ${LIBMODE} ${VIOSIM_CORE_SOURCES})
target_compile_features(viosim_core PUBLIC cxx_std_17)
target_include_directories(viosim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(viosim_core PRIVATE
  VIOSIM_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets/textures")
target_link_libraries(viosim_core PUBLIC PNG::PNG PRIVATE viosim::long_motion)

if(VIOSIM_GPU AND UNIX AND NOT APPLE)
  find_path(VIOSIM_EGL_INCLUDE_DIR EGL/egl.h)
  find_path(VIOSIM_OPENGL_INCLUDE_DIR GL/glcorearb.h)
  find_library(VIOSIM_EGL_LIBRARY EGL)
  find_library(VIOSIM_OPENGL_LIBRARY GL)
  if(VIOSIM_EGL_INCLUDE_DIR AND VIOSIM_OPENGL_INCLUDE_DIR AND
     VIOSIM_EGL_LIBRARY AND VIOSIM_OPENGL_LIBRARY)
    target_compile_definitions(viosim_core PRIVATE VIOSIM_HAVE_EGL_OPENGL)
    target_include_directories(viosim_core PRIVATE
      ${VIOSIM_EGL_INCLUDE_DIR} ${VIOSIM_OPENGL_INCLUDE_DIR})
    target_link_libraries(viosim_core PRIVATE
      ${VIOSIM_EGL_LIBRARY} ${VIOSIM_OPENGL_LIBRARY})
    message(STATUS "VioSim EGL/OpenGL compute renderer enabled")
  else()
    message(WARNING "VioSim GPU renderer disabled: EGL/OpenGL development files not found")
  endif()
endif()

if(VIOSIM_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(viosim_core PUBLIC OpenMP::OpenMP_CXX)
endif()

install(TARGETS viosim_core DESTINATION lib)
install(DIRECTORY include/viosim DESTINATION include)
install(DIRECTORY assets DESTINATION share/viosim)
install(DIRECTORY examples DESTINATION share/viosim)

if(BUILD_TESTING)
  add_executable(test_viosim_imu_frame test_imu_frame.cpp)
  target_link_libraries(test_viosim_imu_frame PRIVATE viosim_core)
  add_test(NAME viosim_imu_frame COMMAND $<TARGET_FILE:test_viosim_imu_frame>)
  add_executable(test_viosim_figure8_forward test_figure8_forward.cpp)
  target_link_libraries(test_viosim_figure8_forward PRIVATE viosim_core)
  add_test(NAME viosim_figure8_forward COMMAND $<TARGET_FILE:test_viosim_figure8_forward>
      ${CMAKE_CURRENT_SOURCE_DIR}/examples/office_figure8_forward_3laps.yaml)
  add_executable(test_viosim_recovery test_recovery.cpp)
  target_link_libraries(test_viosim_recovery PRIVATE viosim_core)
  add_test(NAME viosim_recovery COMMAND $<TARGET_FILE:test_viosim_recovery>)
  set_tests_properties(viosim_recovery PROPERTIES LABELS "unit;viosim;geometry")
  add_executable(test_viosim_multilevel test_multilevel.cpp)
  target_link_libraries(test_viosim_multilevel PRIVATE viosim_core)
  add_test(NAME viosim_multilevel COMMAND $<TARGET_FILE:test_viosim_multilevel>)
  set_tests_properties(viosim_multilevel PROPERTIES LABELS "unit;viosim;geometry")
  add_executable(test_viosim_lab_room test_lab_room.cpp)
  target_link_libraries(test_viosim_lab_room PRIVATE viosim_core)
  add_test(NAME viosim_lab_room COMMAND $<TARGET_FILE:test_viosim_lab_room>
      ${CMAKE_CURRENT_SOURCE_DIR}/assets/textures)
  set_tests_properties(viosim_lab_room PROPERTIES LABELS "integration;viosim")
endif()

if(VIOSIM_BUILD_APPS)
  add_subdirectory(apps)
endif()
