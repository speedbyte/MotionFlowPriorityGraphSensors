PROJECT( DisparityRefinementPlugin )


include_directories(
	${COMMON_INC}
	"${BASE_DIR}/disparityfilter/"
)

set( SRCS 
    "../DisparityRefinementFilter.cpp"	
)

add_definitions(
-DPLUGIN_NAME="Disparity Refinement Filter" 
-DPLUGIN_VERSION="1.0"
-DPLUGIN_AUTHOR="UTue"
-DPLUGIN_IPIN_NAME="Input image"
-DPLUGIN_OPIN_NAME="Image with Disparity Refinement applied"
-DORIGINAL_PLUGIN=DisparityRefinementFilter
-DORIGINAL_PLUGIN_H=<DisparityRefinementFilter.h>)

# ---------------------------------------------------
# Plugins are shared libraries (e.g. .dll on Windows)
# ---------------------------------------------------
add_library( ${PROJECT_NAME} SHARED ${COMMON_SRC} ${SRCS} )

target_link_libraries( ${PROJECT_NAME} ${LINK_LIBRARIES})

INSTALL( TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION . COMPONENT ${PROJECT_NAME}
	LIBRARY DESTINATION . COMPONENT ${PROJECT_NAME}
)