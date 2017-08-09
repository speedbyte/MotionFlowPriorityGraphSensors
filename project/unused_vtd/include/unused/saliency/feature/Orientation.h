//
// Created by geislerd on 12.03.17.
//

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace feature {

            class _OrientationImpl {

            public:
                void orientation(cv::Mat1f in, cv::Mat1f out, float sigma, float theta, float lambda, float gamma);

            };

            template<uint32_t _width, uint32_t _height>
            class _Orientation : public _OrientationImpl, public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
            public:

                float sigma() {
                    const float sigma = this->properties()->template get<float>("sigma",1.0f);

                    return sigma;
                }

                float theta() {
                    const float theta = this->properties()->template get<float>("theta",0.0f);

                    return theta;
                }

                float lambda() {
                    const float lambda = this->properties()->template get<float>("lambda",1.0f);

                    return lambda;
                }

                float gamma() {
                    const float gamma = this->properties()->template get<float>("gamma",1.0f);

                    return gamma;
                }

                void calc() override {
                    cv::Mat1f in, out;

                    in = this->template input<0>()->value()->mat();
                    out = this->map();

                    this->orientation(in,out,this->sigma(),this->theta(),this->lambda(),this->gamma());
                }

                void reset() override {
                }
            };

            typedef _Orientation<RESOLUTION> Orientation;

        }
    }
}

#endif //ORIENTATION_H
