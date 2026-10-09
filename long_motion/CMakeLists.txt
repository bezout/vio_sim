add_library(viosim_long_motion
  src/collision.cpp
  src/options.cpp
  src/planner.cpp
  src/segment.cpp)
add_library(viosim::long_motion ALIAS viosim_long_motion)

target_compile_features(viosim_long_motion PUBLIC cxx_std_17)
target_include_directories(viosim_long_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
if(NOT MSVC)
  target_compile_options(viosim_long_motion PRIVATE -ffp-contract=off)
endif()

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_executable(test_viosim_long_motion tests/test_motion.cpp)
  target_link_libraries(test_viosim_long_motion PRIVATE
    viosim::long_motion Threads::Threads)
  add_test(NAME viosim_long_motion COMMAND $<TARGET_FILE:test_viosim_long_motion>)
  set_tests_properties(viosim_long_motion PROPERTIES LABELS "unit;viosim")
endif()

install(TARGETS viosim_long_motion DESTINATION lib)
install(DIRECTORY include/long_motion DESTINATION include)
install(FILES LICENSE DESTINATION share/licenses/viosim_long_motion)
