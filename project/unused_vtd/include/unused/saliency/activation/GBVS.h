//
// Created by geislerd on 13.03.17.
//

#ifndef GBVS_H
#define GBVS_H

#include <unused/saliency/Saliency.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {


            template<uint32_t _width, uint32_t _height, uint32_t _iheight = 32, uint32_t _iwidth = 32>
            class _GBVS : public _Saliency<_width,_height,vtd_framework::utils::_HeatmapImage<_width,_height>> {
            private:
                float m_adj_dist[_iwidth*_iheight*_iwidth*_iheight];
                cv::Mat1f m_adj_dist_mat;

                float m_adj_weight[_iwidth*_iheight*_iwidth*_iheight];
                cv::Mat1f m_adj_weight_mat;

                float m_in[_iwidth*_iheight];
                cv::Mat1f m_in_mat;

                float m_sigma;
            public:
                _GBVS() :
                        m_adj_dist_mat(_iwidth*_iheight,_iwidth*_iheight,this->m_adj_dist),
                        m_adj_weight_mat(_iwidth*_iheight,_iwidth*_iheight,this->m_adj_weight),
                        m_in_mat(_iheight,_iwidth,this->m_in),
                        m_sigma(0.0f) {
                    this->reset();
                }

                float F(const int& x, const int& y, const float& sigma) {
                    return expf(-(x*x+y*y)/2.0f*sigma*sigma);
                }

                void distAdjacentMat(const float& sigma) {
                    if(sigma != this->m_sigma)
                        for(int x = 0; x < _iwidth; x++) {
                            for (int y = 0; y < _iheight; y++) {
                                for(int xr = 0; xr < _iwidth; xr++) {
                                    for(int yr = 0; yr < _iheight; yr++) {
                                        this->m_adj_dist_mat.template at<float>(y*_iwidth+x,yr*_iwidth+xr) = this->F(x-xr,y-yr,2.0f);
                                    }
                                }
                            }
                        }
                }

                float m_adj0[_iwidth*_iheight*_iwidth*_iheight];
                float m_adj1[_iwidth*_iheight*_iwidth*_iheight];

                void weightAdjacentMat(const float& sigma) {
                    cv::Mat1f v0, v1, adj0(_iheight,_iwidth,m_adj0), adj1(_iheight,_iwidth,m_adj1);

                    this->distAdjacentMat(sigma);

                    v0 = cv::Mat1f(1,_iwidth*_iheight,this->m_in);
                    v1 = cv::Mat1f(_iwidth*_iheight,1,this->m_in);

                    cv::multiply(v0,v0,v0);

                    cv::repeat(v0,_iwidth*_iheight,1,adj0);
                    cv::repeat(v1,1,_iwidth*_iheight,adj1);

                    cv::absdiff(adj0,adj1,this->m_adj_weight_mat);

                    cv::multiply(this->m_adj_weight_mat,this->m_adj_dist_mat,this->m_adj_weight_mat);
                }

                void calc() override {
                    float out[_iwidth*_iheight];
                    float sigma;
                    cv::Mat1f out_mat, v, in, fov, inC, fovC, sal;
                    int n;
                    cv::Point fovMin, fovMax;
                    cv::Rect fovR(_width,_height,0,0);

                    in = this->template input<0>()->value()->mat();
                    fov = this->fov();
                    sal = this->map();
                    n = this->properties()->template get<int>("iters",1);
                    sigma = this->properties()->template get<float>("sigma",4.0f);

                    // find fov rect
                    fovMin = cv::Point(_width,_height);
                    fovMax = cv::Point(0,0);
                    for(int x = 0; x < _width; x++) {
                        for(int y = 0; y < _height; y++) {
                            if(fov.template at<float>(y,x) < 1)
                                continue;
                            fovMin.x = MIN(fovMin.x,x);
                            fovMin.y = MIN(fovMin.y,y);
                            fovMax.x = MAX(fovMax.x,x);
                            fovMax.y = MAX(fovMax.y,y);
                        }
                    }

                    // skip if rect to small
                    fovR = cv::Rect(fovMin,fovMax);

                    if(fovR.width < _iwidth || fovR.height < _iheight) {
                        this->map(0.0f);
                        return;
                    }

                    // cut input to fov rect
                    inC = in(fovR);
                    fovC = fov(fovR);

                    // resize input to internal resolution
                    cv::resize(inC,this->m_in_mat,cv::Size(_iwidth,_iheight),0.0f,0.0f,cv::INTER_AREA);
                    cv::namedWindow("DEBUG");
                    cv::imshow("DEBUG",this->m_in_mat);

                    // calculate adjacemt matrix
                    this->weightAdjacentMat(sigma);


                    //if(!this->properties()->template get<bool>("video",false))
                        this->map(0.0f); // reset old output if not in video mode

                    // resize old output to internal resolution
                    out_mat = cv::Mat1f(_iheight,_iwidth,out);
                    //cv::resize(this->map(),out_mat,cv::Size(_iwidth,_iheight),0.0f,0.0f,cv::INTER_AREA);

                    // initialize solution
                    //out_mat.template at<float>(_iheight/2.0f,_iwidth/2.0f) = 1;
                    //cv::GaussianBlur(out_mat,out_mat,cv::Size(_iwidth*2+1,_iheight*2+1),sigma*2,sigma*2,cv::BORDER_REPLICATE);
                    cv::resize(fovC,out_mat,cv::Size(_iwidth,_iheight),0.0f,0.0f,cv::INTER_AREA);

                    // solve markovian system
                    v = cv::Mat1f(1, _iwidth*_iheight, out);
                    for(int j = 0; j < n; j++) {
                        cv::gemm(v, this->m_adj_weight_mat, 1.0f, cv::noArray(), 0.0f, v);
                        cv::normalize(v,v,0,1,cv::NORM_MINMAX);
                    }

                    // resize solution to external resolution
                    // cv::resize(out_mat,this->map(),cv::Size(_width,_height),0.0f,0.0f,cv::INTER_LINEAR);
                    cv::resize(out_mat,out_mat,fovR.size(),0,0,cv::INTER_LINEAR);

                    for(int x = 0; x < fovR.width; x++) {
                        for(int y = 0; y < fovR.height; y++) {
                            sal.template at<float>(y+fovR.y,x+fovR.x) = out_mat.template at<float>(y,x);
                        }
                    }
                }

                void reset() override {
                }
            };
        }
    }
}

#endif //GBVS_H
