cmake_minimum_required (VERSION 2.6)
project (Tutorial)

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/libs/opencv/include")
include_directories("${PROJECT_BINARY_DIR}")

configure_file (
  "${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
  "${PROJECT_BINARY_DIR}/TutorialConfig.h"
  )

option (USE_MYMATH "Use Tutorial provided math implementation" ON)

if (USE_MYMATH)
add_subdirectory(maths) # This directory must contain a CMakeLists.txt too
set (EXTRA_LIBS ${EXTRA_LIBS} MathFunctions)
endif (USE_MYMATH)

add_executable(Tutorial Tutorial.cpp)
target_link_libraries(Tutorial ${EXTRA_LIBS} )


message(${PROJECT_BINARY_DIR})
