find_package(Doxygen)
message("Doxygen version ${DOXYGEN_FOUND} ${DOXYGEN_VERSION}")

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
endif(DOXYGEN_FOUND)

