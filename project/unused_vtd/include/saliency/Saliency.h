//
// Created by geislerd on 08.02.17.
//

#ifndef SALIENCY_H
#define SALIENCY_H

#include <cstdint>
#include <core/Node.h>
#include <utils/Image.h>

namespace vtd_framework {
    namespace saliency {

        template<uint32_t _width, uint32_t _height>
        class _SaliencyMap : public vtd_framework::utils::_HeatmapImage<_width, _height> {

        };

        template<uint32_t _width, uint32_t _height>
        class _FOVMap : public vtd_framework::utils::_HeatmapImage<_width, _height> {

        };

        template<uint32_t _width, uint32_t _height, typename... _input>
        class _Saliency : public core::Node::
        template Input<_input...,_FOVMap<_width,_height>>::
        template Output<_SaliencyMap<_width, _height>> {
        public:
            typedef _SaliencyMap<_width, _height> SaliencyMap;
            typedef _FOVMap<_width, _height> FOVMap;

        private:
            class DummyFOV : public vtd_framework::core::Node::
            template Input<>::
            template Output<FOVMap> {
            private:
                FOVMap m_map;
            public:
                DummyFOV() {
                    this->template output<0>()->name("fov");
                    this->template output<0>()->value(&this->m_map);
                    this->reset();
                }

                void calc() override { }

                void reset() override {
                    for(int i = 0; i < FOVMap::WIDTH*FOVMap::HEIGHT; this->m_map.template at<float>(i++) = 1.0f);
                }
            };

            DummyFOV m_dummy_fov;

            SaliencyMap m_map;

        protected:

            cv::Mat1f fov() {
                return this->template input<sizeof...(_input)>()->value()->mat();
            }

            cv::Mat1f map() {
                return this->m_map.mat();
            }

            void map(cv::Mat1f map) {
                this->m_map.mat(map);
            }

            void map(float v) {
                cv::Mat1f m;

                m = this->map();
                for(cv::Point xy(0,0); xy.x < m.cols; xy.x++)
                    for(xy.y = 0; xy.y < m.rows; xy.y++)
                        m(xy) = v;
            }

            SaliencyMap* saliencyMap() {
                return &this->m_map;
            }

            void normalize() {
                if (this->properties()->template get<bool>("normalize_output", true))
                    cv::normalize(this->map(), this->map(), 0, 1, cv::NORM_MINMAX);
            }

            void attenuate() {
                if (!this->properties()->template get<bool>("attenuate_border", true))
                    return;
                //TODO: Not implemented
            }

        public:
            _Saliency() {
                this->template output<0>()->name("saliency");
                this->template output<0>()->value(&(this->m_map));

                this->template input<0>()->name("feature");
                this->template input<sizeof...(_input)>()->name("fov");
                this->template input<sizeof...(_input)>()->connect(this->m_dummy_fov.template output<0>());
            };
        };
    }
}
#endif //SALIENCY_H
