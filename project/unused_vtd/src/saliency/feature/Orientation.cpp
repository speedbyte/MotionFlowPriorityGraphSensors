//
// Created by geislerd on 12.03.17.
//

#include <unused/saliency/feature/Orientation.h>

namespace vtd_framework {
    namespace saliency {
        namespace feature {
            void _OrientationImpl::orientation(cv::Mat1f in, cv::Mat1f out, float sigma, float theta, float lambda, float gamma) {
                cv::Mat1f filter;

                filter = cv::getGaborKernel(cv::Size(33,33),sigma,theta,lambda,gamma);

                cv::filter2D(in,out,-1,filter,cv::Point(-1,-1),0,cv::BORDER_REPLICATE);
            }
        }
    }
}