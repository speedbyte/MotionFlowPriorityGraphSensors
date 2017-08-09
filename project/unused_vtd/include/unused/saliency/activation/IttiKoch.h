//
// Created by geislerd on 10.03.17.
//

#ifndef ITTIKOCH_H
#define ITTIKOCH_H

#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            template<uint32_t _width, uint32_t _height>
            class _IttiKoch : public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
            public:

                template<uint32_t _c, uint32_t _delta>
                class CenterSurroundDifference : public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
                public:

                    cv::Mat1f calc(cv::Mat1f in) {
                        cv::Mat1f fine, coarse, out;
                        float v, m, M, f;
                        int c;

                        out = this->map();

                        cv::resize(in,fine,cv::Size(_width/_c,_height/_c));
                        cv::resize(fine,fine,cv::Size(_width,_height));

                        cv::resize(in,coarse,cv::Size(_width/(_c+_delta),_height/(_c+_delta)));
                        cv::resize(coarse,coarse,cv::Size(_width,_height));

                        cv::absdiff(coarse,fine,out);

                        m = 0;
                        c = 0;
                        M = FLT_MIN;
                        for(int x = 1; x < _width-1; x++) {
                            for(int y = 1; y < _height-1; y++) {
                                v = out.template at<float>(y,x);
                                if(v < out.template at<float>(y-1,x-1))
                                    continue;
                                if(v < out.template at<float>(y,x-1))
                                    continue;
                                if(v < out.template at<float>(y+1,x-1))
                                    continue;
                                if(v < out.template at<float>(y-1,x))
                                    continue;
                                if(v < out.template at<float>(y+1,x))
                                    continue;
                                if(v < out.template at<float>(y-1,x-1))
                                    continue;
                                if(v < out.template at<float>(y,x+1))
                                    continue;
                                if(v < out.template at<float>(y+1,x-1))
                                    continue;
                                if(isnanf(v))
                                    continue;
                                m += v;
                                c++;
                                M = MAX(M,v);
                            }
                        }
                        f = M-(m/c);
                        cv::multiply(out,cv::Scalar(f*f),out);

                        return out;
                    }

                    void calc() override {
                        this->calc(this->template input<0>()->value()->mat());
                    }

                    void reset() override {

                    }
                };

            private:
                CenterSurroundDifference<2,3> m_csd_0;
                CenterSurroundDifference<2,4> m_csd_1;
                CenterSurroundDifference<3,3> m_csd_2;
                CenterSurroundDifference<3,4> m_csd_3;
                CenterSurroundDifference<4,3> m_csd_4;
                CenterSurroundDifference<4,4> m_csd_5;

            public:
                _IttiKoch() {
                }

                void calc() override {
                    cv::Mat1f in, infov, fov;

                    in = this->template input<0>()->value()->mat();
                    fov = this->fov();

                    cv::multiply(in,fov,infov);

                    this->map(0.0f);

                    cv::scaleAdd(this->m_csd_0.calc(infov),1.0f/6.0f,this->map(),this->map());
                    cv::scaleAdd(this->m_csd_1.calc(infov),1.0f/6.0f,this->map(),this->map());
                    cv::scaleAdd(this->m_csd_2.calc(infov),1.0f/6.0f,this->map(),this->map());
                    cv::scaleAdd(this->m_csd_3.calc(infov),1.0f/6.0f,this->map(),this->map());
                    cv::scaleAdd(this->m_csd_4.calc(infov),1.0f/6.0f,this->map(),this->map());
                    cv::scaleAdd(this->m_csd_5.calc(infov),1.0f/6.0f,this->map(),this->map());

                    cv::multiply(this->map(),fov,this->map());
                }

                void reset() override {

                }
            };

            typedef _IttiKoch<RESOLUTION> IttiKoch;
        }
    }
}
#endif //ITTIKOCH_H
