cmake_minimum_required(VERSION 3.2.0)

project(polytri VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)

# -----------------------------------------------------------------------------

set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

# -----------------------------------------------------------------------------

if(   (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
   OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  list(APPEND CXX_FLAGS 
    -Wall -Wextra -Wshadow -Wpedantic
    -Wnon-virtual-dtor -Woverloaded-virtual
    -Wuseless-cast -Wcast-align
    #-Wold-style-cast
    -Wlogical-op
    -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers
    -fvisibility=hidden -fno-strict-aliasing -fno-builtin-memcmp
  )
  list(APPEND CXX_FLAGS_RELEASE -O2)
  list(APPEND CXX_FLAGS_DEBUG   -g -O0 -Wconversion)
else()
  message(WARNING "Compiler not tested")
endif()

# -----------------------------------------------------------------------------

set(Sources
  ${SOURCE_DIR}/trapezoidal_decomposition.cc
  ${SOURCE_DIR}/monotone_partitioning.cc
  ${SOURCE_DIR}/polygon_triangulation.cc
)

set(Headers
  ${SOURCE_DIR}/polygon_triangulation.h
)

# -----------------------------------------------------------------------------

add_library(${PROJECT_NAME} STATIC ${Sources} ${Headers})
target_compile_options(${PROJECT_NAME} PRIVATE
  "${CXX_FLAGS}"
  "$<$<CONFIG:Debug>:${CXX_FLAGS_DEBUG}>"
  "$<$<CONFIG:Release>:${CXX_FLAGS_RELEASE}>"
)
target_include_directories(${PROJECT_NAME} PRIVATE ${SOURCE_DIR})

# -----------------------------------------------------------------------------

set(TargetName polytri-cli)
add_executable(${TargetName}
  ${SOURCE_DIR}/polytri-cli.cc
  ${Headers}
)
target_include_directories(${TargetName} PRIVATE ${SOURCE_DIR} )
target_link_libraries(${TargetName} ${PROJECT_NAME})
target_compile_definitions(${TargetName} PRIVATE -DAPP_DIRECTORY="${CMAKE_CURRENT_LIST_DIR}/")

# -----------------------------------------------------------------------------
