//
// Created by geislerd on 15.03.17.
//

#ifndef TRACKING_H
#define TRACKING_H

#include <utils/Image.h>
#include <flow/Flow.h>
#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {

        template<uint32_t _width, uint32_t _height>
        class _Tracking : public _Saliency<_width,_height,_SaliencyMap<_width,_height>,vtd_framework::flow::_FlowField<_width,_height>> {
        private:
            typedef vtd_framework::flow::_FlowField<_width,_height> FlowField;

            SaliencyMap m_prev;
        public:
            _Tracking() {
                this->template input<0>()->name("feature");
                this->template input<1>()->name("flow");
            }

            void calc() override {
                SaliencyMap *next, *prev, *trac;
                FlowField *flow;
                cv::Point p[2];
                SaliencyMap::ValueType v[2];
                FlowField::ValueType f;

                prev = &this->m_prev;
                next = this->template input<0>()->value();
                flow = this->template input<1>()->value();
                trac = this->saliencyMap();

                // cleanup old map
                this->map(0.0f);

                for(p[0] = cv::Point(0,0); p[0].x < _width; p[0].x++) {
                    for(p[0].y = 0; p[0].y < _height; p[0].y++) {
                        // flow value at point p[0]
                        f = flow->template at<FlowField::ValueType>(p[0]);
                        // previous point p[1]
                        p[1] = cv::Point(int(p[0].x-f.val[0]),int(p[0].y-f.val[1]));
                        // skip if we outside of the image bounds
                        if(!cv::Rect(0,0,_width,_height).contains(p[1]))
                            continue;
                        // saliency value at point p[0]
                        v[0] = next->template at<SaliencyMap::ValueType>(p[0]);
                        // saliency value at point p[1]
                        v[1] = next->template at<SaliencyMap::ValueType>(p[1]);

                        trac->template at<SaliencyMap::ValueType>(p[0]) = 0.5f*sqrtf(fabsf(v[0]*v[1]));
                    }
                }

                // store current saliency for next iteration
                this->m_prev.mat(*next);
            }

            void reset() override {

            }
        };
    }
}

#endif //TRACKING_H
