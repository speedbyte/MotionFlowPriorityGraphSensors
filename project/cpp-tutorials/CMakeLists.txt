cmake_minimum_required(VERSION 3.5)
project(cpp_tutorials)

set(CMAKE_CXX_STANDARD 11)

find_package(Doxygen)

#set(SOURCE_FILES main2.cpp)
#add_library( tutorials.lib ${SOURCE_FILES})

set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/../../libs/boost-install/include")
find_package(Boost COMPONENTS system filesystem)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(overloading main.cpp Overload.cpp Larger.cpp Template.cpp OperatorOverload.cpp)
target_link_libraries(overloading ${Boost_LIBRARIES})

message("Doxygen found ${DOXYGEN_FOUND} ${DOXYGEN_VERSION}")

if(DOXYGEN_FOUND)

    set(DOXYGEN_INPUT main.cpp)
#    set(DOXYGEN_INPUT {SOURCE_FILES})
    set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    add_custom_command(
            OUTPUT ${DOXYGEN_OUTPUT}
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
            COMMAND ${CMAKE_COMMAND} -E echo "Done."
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS ${DOXYGEN_INPUT}
    )

    add_custom_target(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT})

    add_custom_target(apidoc_forced
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
            COMMAND ${CMAKE_COMMAND} -E echo "Done."
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            )

endif(DOXYGEN_FOUND)


