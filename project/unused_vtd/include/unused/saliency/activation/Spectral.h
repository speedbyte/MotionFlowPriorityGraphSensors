//
// Created by geislerd on 09.02.17.
//

#ifndef SPECTRAL_H_H
#define SPECTRAL_H_H

#include <cstdint>
#include <config.h>
#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            class _SpectralImpl {
            protected:
                cv::Mat1f wrap2pi(cv::Mat1f in);
                cv::Mat1f sin(cv::Mat1f in);
                cv::Mat1f cos(cv::Mat1f in);
                void spectralWhitening(cv::Mat1f in, cv::Mat1f out, cv::Size fz, float whitening, float smooth);
            };

            template<uint32_t _width, uint32_t _height>
            class _Spectral : public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>>,
                              public _SpectralImpl {
            public:

                cv::Size filterSize() {
                    const int x = this->properties()->template get<int>("filter_size_x",8);
                    const int y = this->properties()->template get<int>("filter_size_y",8);

                    return cv::Size(x*2+1,y*2+1);
                }

                float whiteningSigma() {
                    const float s = this->properties()->template get<float>("whitening_sigma",0.0f);

                    return s;
                }

                float smoothSigma() {
                    const float s = this->properties()->template get<float>("smooth_sigma",4.0f);

                    return s;
                }

                void calc() override {
                    this->spectralWhitening(this->input()->value()->mat(),this->map(),this->filterSize(),this->whiteningSigma(),this->smoothSigma());

                    this->normalize();
                    this->attenuate();

                    cv::multiply(this->map(),this->fov(),this->map());
                }

                void reset() override {

                }
            };

            typedef _Spectral<RESOLUTION> Spectral;

        }
    }
}

#endif //SPECTRAL_H_H
