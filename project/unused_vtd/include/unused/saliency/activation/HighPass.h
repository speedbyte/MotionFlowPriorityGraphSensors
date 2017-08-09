//
// Created by geislerd on 09.03.17.
//

#ifndef HIGHPASS_H
#define HIGHPASS_H

#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            class _HighPassImpl {
            protected:
                void highpass(cv::Mat1f in, cv::Mat1f out, cv::Size sz, float sigma);
            };

            template<uint32_t _width, uint32_t _height>
            class _HighPass : public _HighPassImpl, public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
            public:

                float sigma() {
                    const float s = this->properties()->template get<float>("sigma",16.0f);

                    return s;
                }

                cv::Size filterSize() {
                    const int x = this->properties()->template get<int>("filter_size",21);

                    return cv::Size(x,x);
                }

                void calc() override {
                    this->highpass(this->template input<0>()->value()->mat(),this->map(),this->filterSize(),this->sigma());
                    cv::multiply(this->fov(),this->map(),this->map());
                    this->normalize();
                }

                void reset() override {

                }
            };


            typedef _HighPass<RESOLUTION> HighPass;
        }
    }
}

#endif //HIGHPASS_H
