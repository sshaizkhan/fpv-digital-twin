# The physics core. Phase 0 ships only the config loader; the 6DOF integrator,
# motor/battery models, IMU synthesis and the SITL/SDL2/viewer I/O land in
# later phases and get added here.

add_library(fdt_physics
  src/quad_config.cpp
  src/rigid_body.cpp
  src/battery.cpp
  src/motor.cpp
  src/aero.cpp
  src/ground.cpp
  src/imu.cpp
  src/multirotor.cpp
  src/sitl_link.cpp
  src/msp_client.cpp
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

add_executable(fdt_sim apps/sim_step.cpp)
target_link_libraries(fdt_sim PRIVATE fdt_physics fdt_warnings)

add_executable(fdt_sitl_probe apps/sitl_probe.cpp)
target_link_libraries(fdt_sitl_probe PRIVATE fdt_physics fdt_warnings)

# --- tests -----------------------------------------------------------------

if(FDT_BUILD_TESTS)
  add_executable(fdt_physics_tests
    tests/test_quad_config.cpp
    tests/test_rigid_body.cpp
    tests/test_propulsion.cpp
    tests/test_aero_ground.cpp
    tests/test_imu.cpp
    tests/test_multirotor.cpp
    tests/test_sitl_packets.cpp
    tests/test_sitl_link.cpp
  )
  target_link_libraries(fdt_physics_tests PRIVATE fdt_physics GTest::gtest_main fdt_warnings)
  target_compile_definitions(fdt_physics_tests PRIVATE FDT_REPO_ROOT="${FDT_REPO_ROOT}")

  include(GoogleTest)
  gtest_discover_tests(fdt_physics_tests
    PROPERTIES LABELS physics
    DISCOVERY_MODE PRE_TEST
  )


  # --- CLI behaviour ---------------------------------------------------------
  # fdt_sim's step rate must come from sim.physics_rate in the config, not from
  # a literal in main(). Pinned here because it is main() wiring, not library
  # behaviour the gtest binary can reach. If sim.physics_rate in
  # config/quad.yaml changes, this regex changes with it -- deliberately, so
  # the coupling stays visible.
  add_test(NAME fdt_sim.DefaultsToTheConfiguredPhysicsRate
    COMMAND fdt_sim --config "${FDT_REPO_ROOT}/config/quad.yaml" --duration 0.01
  )
  set_tests_properties(fdt_sim.DefaultsToTheConfiguredPhysicsRate PROPERTIES
    PASS_REGULAR_EXPRESSION "at 8000\\.000 Hz"
    LABELS physics
  )

  add_test(NAME fdt_sim.ExplicitRateOverridesTheConfig
    COMMAND fdt_sim --config "${FDT_REPO_ROOT}/config/quad.yaml" --duration 0.01 --rate 1000
  )
  set_tests_properties(fdt_sim.ExplicitRateOverridesTheConfig PROPERTIES
    PASS_REGULAR_EXPRESSION "at 1000\\.000 Hz"
    LABELS physics
  )

  # A rate of zero used to run nothing, print `inf us/step` and exit 0.
  add_test(NAME fdt_sim.RejectsANonPositiveRate
    COMMAND fdt_sim --config "${FDT_REPO_ROOT}/config/quad.yaml" --rate 0
  )
  set_tests_properties(fdt_sim.RejectsANonPositiveRate PROPERTIES
    WILL_FAIL TRUE
    LABELS physics
  )

  add_test(NAME fdt_sim.RejectsANonPositiveDuration
    COMMAND fdt_sim --config "${FDT_REPO_ROOT}/config/quad.yaml" --duration 0
  )
  set_tests_properties(fdt_sim.RejectsANonPositiveDuration PROPERTIES
    WILL_FAIL TRUE
    LABELS physics
  )
endif()
