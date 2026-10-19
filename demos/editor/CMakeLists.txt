cmake_minimum_required(VERSION 3.18)

set(target polytri-editor)
project(${target} CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------

include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# -----------------------------------------------------------------------------

add_executable(${target} main.cc)
target_include_directories(
  ${target}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)
target_link_libraries(${target} PRIVATE raylib)

# -----------------------------------------------------------------------------
