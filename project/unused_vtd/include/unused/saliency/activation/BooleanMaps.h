//
// Created by geislerd on 25.02.17.
//

#ifndef BOOLEANMAPS_H
#define BOOLEANMAPS_H

#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            class _BooleanMapsImpl {
            public:

                void floodFillBorder(cv::Mat1f& map);

                void add(cv::Mat1f map, cv::Mat1f out);

                void threshold(cv::Mat1f mat, cv::Mat1f out, double hmin, double hmax, int n, int type);

                void booleanMaps(cv::Mat1f in, cv::Mat1f out, cv::Size sz, float sigma, int n);
            };

            template<uint32_t _width, uint32_t _height>
            class _BooleanMaps : public _BooleanMapsImpl, public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
            public:
                _BooleanMaps() {
                    this->reset();
                }

                cv::Size filterSize() {
                    const int x = this->properties()->template get<int>("filter_size",8);

                    return cv::Size(x*2+1,x*2+1);
                }

                int numThresholds() {
                    const int n = this->properties()->template get<int>("num_thresholds",3);

                    return n;
                }

                float smoothSigma() {
                    const float s = this->properties()->template get<float>("smooth_sigma",3.0f);

                    return s;
                }

                void calc() override {
                    this->map(0.0f);
                    this->booleanMaps(this->template input<0>()->value()->mat(),this->map(),this->filterSize(),this->smoothSigma(),this->numThresholds());
                    cv::multiply(this->map(),this->fov(),this->map());
                }

                void reset() override {

                }
            };

            typedef _BooleanMaps<RESOLUTION> BooleanMaps;
        }
    }
}
#endif //BOOLEANMAPS_H
