//
// Created by geislerd on 10.03.17.
//

#include <utils/Image.h>

#ifndef PEARSON_H
#define PEARSON_H

namespace vtd_framework {
    namespace saliency {
        namespace evaluation {

            class _PearsonImpl {
            protected:

                void pearson(cv::Mat1f saliency, cv::Mat1f fixation,float& correl);

            };

            template<uint32_t _width, uint32_t _height>
            class _Pearson : public _PearsonImpl,public vtd_framework::core::Node::template Input<vtd_framework::utils::_HeatmapImage<_width,_height>,vtd_framework::utils::_HeatmapImage<_width,_height>>::template Output<float> {
            private:
                float m_correl;
            public:
                _Pearson() {
                    this->template input<0>()->name("saliency");
                    this->template input<1>()->name("ground truth");
                    this->template output<0>()->name("pearson");
                    this->template output<0>()->value(&this->m_correl);
                }

                void calc() override {
                    cv::Mat1f saliency;
                    cv::Mat1f fixation;

                    saliency = this->template input<0>()->value()->mat();
                    fixation = this->template input<1>()->value()->mat();

                    this->pearson(saliency,fixation,this->m_correl);
                }

                void reset() override {

                }
            };

            typedef _Pearson<RESOLUTION> Pearson;
        }
    }
}

#endif //PEARSON_H
