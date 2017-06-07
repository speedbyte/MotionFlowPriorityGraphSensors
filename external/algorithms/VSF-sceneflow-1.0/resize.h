
#ifndef _RESIZE_H_
#define _RESIZE_H_

#include "cxcore.h"
#include "cvtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef MY_CVRESIZESCALE
/* Resizes image (input array is resized to fit the destination array) */
CVAPI(void)  cvResizeScale( const CvArr* src, CvArr* dst,
                       int interpolation CV_DEFAULT( CV_INTER_LINEAR ),
                       float scale_x CV_DEFAULT( 0. ),
                       float scale_y CV_DEFAULT( 0. ));
#else
#define cvResizeScale(src,dst,method,scale_x,scale_y) cvResize(src,dst,method)
#endif


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "cv.hpp"
#endif

#endif /*_RESIZE_H_*/
