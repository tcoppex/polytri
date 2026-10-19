cmake_minimum_required(VERSION 3.18)

project(polytri_demos VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------

option(POLYTRI_BUILD_EDITOR       "Build the raylib editor"                         ON)
option(POLYTRI_BUILD_BENCHMARK    "Build the benchmark against other libraries"     ON)

if(POLYTRI_BUILD_EDITOR)
  add_subdirectory(editor)
endif()

if(POLYTRI_BUILD_BENCHMARK)
  add_subdirectory(benchmark)
endif()

# -----------------------------------------------------------------------------