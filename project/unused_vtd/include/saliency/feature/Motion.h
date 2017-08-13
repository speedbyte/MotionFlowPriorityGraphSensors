//
// Created by geislerd on 20.03.17.
//

#ifndef MOTION_H
#define MOTION_H

#include <unused/saliency/Saliency.h>
#include <flow/Flow.h>

namespace vtd_framework {
    namespace saliency {
        namespace feature {
            template<uint32_t _width, uint32_t _height>
            class _Orientation : public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>,vtd_framework::flow::_FlowField<_width,_height>> {
            private:
                const static float F1[5*5] = {
                        1.0f,1.0f,1.0f,1.0f,1.0f,
                        2.0f,2.0f,2.0f,2.0f,2.0f,
                        -5.0f,-5.0f,-5.0f,-5.0f,-5.0f,
                        1.0f,1.0f,1.0f,1.0f,1.0f,
                        2.0f,2.0f,2.0f,2.0f,2.0f};
                const static float F2[5*5] = {
                        1.0f,2.0f,-5.0f,2.0f,1.0f,
                        1.0f,2.0f,-5.0f,2.0f,1.0f,
                        1.0f,2.0f,-5.0f,2.0f,1.0f,
                        1.0f,2.0f,-5.0f,2.0f,1.0f,
                        1.0f,2.0f,-5.0f,2.0f,1.0f};
                const static float F3[5*5] = {
                        -5.0f,2.0f,1.0f,2.0f,-5.0f,
                        2.0f,2.0f,1.0f,2.0f,2.0f,
                        1.0f,1.0f,1.0f,1.0f,1.0f,
                        2.0f,2.0f,1.0f,2.0f,2.0f,
                        -5.0f,2.0f,1.0f,2.0f,-5.0f};
            public:
                void calc() override {

                }

                void reset() override {

                }
            };
        }
    }
}
#endif //MOTION_H
