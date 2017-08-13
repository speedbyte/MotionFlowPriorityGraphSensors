MESSAGE("-- create saliency library target")

aux_source_directory(. SALIENCY_SOURCE)
aux_source_directory(activation ACTIVATION_SOURCE)
aux_source_directory(evaluation EVALUATION_SOURCE)
aux_source_directory(feature FEATURE_SOURCE)
MESSAGE("   source files: ${SALIENCY_SOURCE} ${ACTIVATION_SOURCE} ${EVALUATION_SOURCE} ${FEATURE_SOURCE}")

add_library(saliency SHARED ${SALIENCY_SOURCE} ${ACTIVATION_SOURCE} ${EVALUATION_SOURCE} ${FEATURE_SOURCE})
target_link_libraries( saliency core io utils ${OpenCV_LIBS} ${Boost_LIBRARIES} ${EXTERNAL_LIBS} )

install(TARGETS saliency DESTINATION lib)