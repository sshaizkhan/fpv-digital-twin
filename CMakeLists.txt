cmake_minimum_required(VERSION 3.24)

project(fpv_digital_twin
  VERSION 0.1.0
  DESCRIPTION "Digital twin FPV simulator: real Betaflight SITL flying a measured quad model"
  LANGUAGES CXX
)

if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
  endif()
endif()

option(FDT_BUILD_TESTS "Build the physics unit tests" ON)
option(FDT_WERROR "Treat warnings as errors" OFF)

# Subprojects fetched by FetchContent must not inherit our strict flags.
add_library(fdt_warnings INTERFACE)
target_compile_options(fdt_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion
  $<$<BOOL:${FDT_WERROR}>:-Werror>
)

# The repo root, so tests and tools can find config/quad.yaml regardless of
# the working directory they are run from.
set(FDT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

include(cmake/Dependencies.cmake)

if(FDT_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(physics)

message(STATUS "")
message(STATUS "fpv-digital-twin ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  tests      : ${FDT_BUILD_TESTS}")
message(STATUS "  repo root  : ${FDT_REPO_ROOT}")
message(STATUS "")
