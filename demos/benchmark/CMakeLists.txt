cmake_minimum_required(VERSION 3.18)

set(target polytri-benchmark)
project(${target} CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------

include(FetchContent)

function(fetch name repo tag)
  FetchContent_Declare(${name} GIT_REPOSITORY ${repo} GIT_TAG ${tag} SOURCE_SUBDIR no-cmake)
  FetchContent_MakeAvailable(${name})
  FetchContent_GetProperties(${name} SOURCE_DIR dir)
  set(${name}_SOURCE_DIR ${dir} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------

fetch(earcut      https://github.com/mapbox/earcut.hpp.git    v3.2.4)
fetch(cdt         https://github.com/artem-ogre/CDT.git       2.0.1)
fetch(poly2tri    https://github.com/jhasse/poly2tri.git      1008179917fd38c1824b8fe810f020fef0e5f6b5)

# poly2tri is compiled here.
add_library(thirdparty STATIC
  ${poly2tri_SOURCE_DIR}/poly2tri/common/shapes.cc
  ${poly2tri_SOURCE_DIR}/poly2tri/sweep/advancing_front.cc
  ${poly2tri_SOURCE_DIR}/poly2tri/sweep/cdt.cc
  ${poly2tri_SOURCE_DIR}/poly2tri/sweep/sweep.cc
  ${poly2tri_SOURCE_DIR}/poly2tri/sweep/sweep_context.cc)
target_include_directories(thirdparty PUBLIC ${poly2tri_SOURCE_DIR})

# -----------------------------------------------------------------------------

add_executable(${target} main.cc)
target_include_directories(
  ${target}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${earcut_SOURCE_DIR}/include
    ${cdt_SOURCE_DIR}/CDT/include
)
target_link_libraries(${target} PRIVATE thirdparty)

# -----------------------------------------------------------------------------