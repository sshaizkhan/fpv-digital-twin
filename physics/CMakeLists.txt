# The physics core. Phase 0 ships only the config loader; the 6DOF integrator,
# motor/battery models, IMU synthesis and the SITL/SDL2/viewer I/O land in
# later phases and get added here.

add_library(fdt_physics
  src/quad_config.cpp
)

target_include_directories(fdt_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(fdt_physics
  PUBLIC  Eigen3::Eigen
  PRIVATE yaml-cpp::yaml-cpp fdt_warnings
)

add_library(fdt::physics ALIAS fdt_physics)

# --- apps ------------------------------------------------------------------

add_executable(fdt_config_dump apps/config_dump.cpp)
target_link_libraries(fdt_config_dump PRIVATE fdt_physics fdt_warnings)

# --- tests -----------------------------------------------------------------

if(FDT_BUILD_TESTS)
  add_executable(fdt_physics_tests
    tests/test_quad_config.cpp
  )
  target_link_libraries(fdt_physics_tests PRIVATE fdt_physics GTest::gtest_main fdt_warnings)
  target_compile_definitions(fdt_physics_tests PRIVATE FDT_REPO_ROOT="${FDT_REPO_ROOT}")

  include(GoogleTest)
  gtest_discover_tests(fdt_physics_tests
    PROPERTIES LABELS physics
    DISCOVERY_MODE PRE_TEST
  )
endif()
