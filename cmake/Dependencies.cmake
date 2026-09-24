# Third-party dependencies.
#
# Each uses find_package first (so a Homebrew install is used when present) and
# falls back to a pinned source fetch, so a clean clone builds with no extra
# steps beyond `brew install cmake eigen`.

include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

# --- Eigen (header only) ---------------------------------------------------
# Homebrew: `brew install eigen`
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
  FIND_PACKAGE_ARGS NAMES Eigen3
)

# --- yaml-cpp --------------------------------------------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        yaml-cpp-0.9.0  # 0.8.0 declares cmake_minimum_required(3.4), rejected by CMake 4
  GIT_SHALLOW    TRUE
  FIND_PACKAGE_ARGS NAMES yaml-cpp
)

FetchContent_MakeAvailable(Eigen3 yaml-cpp)

# Eigen 3.4 built from source exports Eigen3::Eigen; Homebrew's Eigen 5 does
# too. Normalise anyway so the rest of the build only ever names one target.
if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3::Eigen target not found after dependency resolution")
endif()

# yaml-cpp 0.8 exports yaml-cpp::yaml-cpp; older/Homebrew builds export yaml-cpp.
if(NOT TARGET yaml-cpp::yaml-cpp)
  if(TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
  else()
    message(FATAL_ERROR "no yaml-cpp target found after dependency resolution")
  endif()
endif()

# --- SDL2 (radio input) ----------------------------------------------------
# Optional: everything except the radio builds and tests without it, so a
# machine with no SDL2 (or a CI box with no joystick) is not blocked.
# Homebrew: `brew install sdl2`
find_package(SDL2 QUIET)
if(SDL2_FOUND)
  set(FDT_HAVE_SDL2 TRUE)
  message(STATUS "SDL2 found: radio input will be built")
else()
  set(FDT_HAVE_SDL2 FALSE)
  message(STATUS "SDL2 NOT found: skipping radio input (brew install sdl2)")
endif()

# --- GoogleTest (tests only) ----------------------------------------------
if(FDT_BUILD_TESTS)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
    FIND_PACKAGE_ARGS NAMES GTest
  )
  FetchContent_MakeAvailable(googletest)
endif()

# --- keep third-party warnings out of our build ----------------------------
# Their headers are included by our sources, so without this every -Wconversion
# in yaml-cpp or googletest drowns out warnings in our own code (and -Werror in
# the release preset would fail on code we do not control).
function(fdt_mark_include_dirs_system target)
  if(NOT TARGET ${target})
    return()
  endif()
  get_target_property(_aliased ${target} ALIASED_TARGET)
  if(_aliased)
    set(target ${_aliased})
  endif()
  get_target_property(_imported ${target} IMPORTED)
  if(_imported)
    return()  # imported targets already present their includes as SYSTEM
  endif()
  get_target_property(_dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
  if(_dirs)
    set_target_properties(${target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_dirs}")
  endif()
endfunction()

foreach(_tp yaml-cpp::yaml-cpp Eigen3::Eigen GTest::gtest GTest::gtest_main GTest::gmock)
  fdt_mark_include_dirs_system(${_tp})
endforeach()
