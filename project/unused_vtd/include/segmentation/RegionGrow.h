//
// Created by geislerd on 06.03.17.
//

#ifndef REGIONGROW_H
#define REGIONGROW_H

#include <core/Node.h>
#include <utils/Image.h>

namespace vtd_framework {
    namespace segmentation {

        using namespace vtd_framework::utils;

        static std::vector<cv::Scalar> generateColors(size_t n) {
            std::vector<cv::Scalar> colors;
            cv::generateColors(colors,n);
            return colors;
        }

        static std::vector<cv::Scalar> colors = generateColors(100);

        template <uint32_t _width, uint32_t _height>
        class _SegmentedImage : public vtd_framework::utils::_Matrix<_width,_height,uint32_t> {
        private:

        public:
            define_image_converter(Segmented, RGB);
            define_image_converter(Segmented, LAB);
            define_image_converter(Segmented, LMS);
            define_image_converter(Segmented, DKL);
            define_image_converter(Segmented, HSV);
            define_image_converter(Segmented, XYZ);
            define_image_converter(Segmented, Intensity);
            define_image_converter(Segmented, Binary);
            define_image_converter(Segmented, Heatmap);

            void convert(_RGBImage<_width,_height>* out) {
                cv::Mat_<uint32_t> in;
                cv::Mat3b rgb;
                cv::Scalar color;
                uint32_t seg;

                in = this->mat();
                rgb = out->mat();

                for(int i = 0; i < _width*_height; i++) {
                    seg = in.at<uint32_t>(i);
                    color = colors[seg % 100];
                    rgb.at<cv::Vec3b>(i) = cv::Vec3b(color.val[0],color.val[1],color.val[2]);
                }
            }
            void convert(_LABImage<_width,_height>* out) {
                _RGBImage<_width,_height> rgb;
                this->convert(&rgb);
                rgb.convert(out);
            }
            void convert(_LMSImage<_width,_height>* out) {
                _RGBImage<_width,_height> rgb;
                this->convert(&rgb);
                rgb.convert(out);
            }
            void convert(_DKLImage<_width,_height>* out) {
                _RGBImage<_width,_height> rgb;
                this->convert(&rgb);
                rgb.convert(out);
            }
            void convert(_HSVImage<_width,_height>* out) {
                _RGBImage<_width,_height> rgb;
                this->convert(&rgb);
                rgb.convert(out);
            }
            void convert(_XYZImage<_width,_height>* out) {
                _RGBImage<_width,_height> rgb;
                this->convert(&rgb);
                rgb.convert(out);
            }
            void convert(_IntensityImage<_width,_height>* out) {
                cv::normalize(*this,*out,0,255,cv::NORM_MINMAX,CV_8UC1);
            }
            void convert(_BinaryImage<_width,_height>* out) {
                _HeatmapImage<_width,_height> heatmap;
                this->convert(&heatmap);
                heatmap.convert(out);
            }
            void convert(_HeatmapImage<_width,_height>* out) {
                float* data_out = (float*) out->data();
                uint32_t* data_in = (uint32_t*)this->data();

                for(int i = 0; i < _width*_height; i++)
                    data_out[i] = data_in[i] / float(_width*_height);
            }
        };

        template <uint32_t _width, uint32_t _height, typename _type>
        class IEdge {
        public:
            virtual _type* v0() = 0;
            virtual _type* v1() = 0;
            virtual uint32_t* s0() = 0;
            virtual uint32_t* s1() = 0;
            virtual uint32_t i0() = 0;
            virtual uint32_t i1() = 0;
            virtual float w() = 0;
        };

        template <typename _type>
        float calcEdgeWeight(_type& v0, _type& v1);

        /**
         *
         * @tparam _i index of voxel 0
         * @tparam _j index of voxel 1
         * @tparam _width width of voxel map
         * @tparam _height height of voxel map
         * @tparam _type of voxel
         */
        template <uint32_t _width, uint32_t _height, typename _type>
        class _Edge : public IEdge<_width,_height,_type> {
        private:
            vtd_framework::utils::_Matrix<_width,_height,_type>** m_data;
            _SegmentedImage<_width,_height>* m_seg;
            uint32_t m_i;
            uint32_t m_j;

        public:
            _Edge(_SegmentedImage<_width,_height>* seg,vtd_framework::utils::_Matrix<_width,_height,_type>** data, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1) :
                    m_data(data),
                    m_seg(seg),
                    m_i(y0*_width+x0),
                    m_j(y1*_width+x1){

            }

            _type* data() {
                return (_type*) this->m_data[0]->data();
            }

            uint32_t* seg() {
                return (uint32_t*) this->m_seg->data();
            }

            _type* v0() override {
                return (_type*) &(this->data()[this->m_i]);
            }

            _type* v1() override {
                return (_type*) &(this->data()[this->m_j]);
            }

            uint32_t *s0() override {
                return (uint32_t *) &(this->seg()[this->m_i]);
            }

            uint32_t *s1() override {
                return (uint32_t *) &(this->seg()[this->m_j]);
            }

            uint32_t i0() override {
                return this->m_i;
            }

            uint32_t i1() override {
                return this->m_j;
            }

            float w() override {
                //return calcEdgeWeight<_type>(this->v0(),this->v1());
            }
        };

        template <uint32_t _width, uint32_t _height, typename _type>
        class _RegionGrow : public vtd_framework::core::Node::template Input<vtd_framework::utils::_Matrix<_width,_height,_type>>::template Output<_SegmentedImage<_width,_height>> {
        public:
            typedef _SegmentedImage<_width,_height> Segmentation;
            typedef _Edge<_width,_height,_type> Edge;
            typedef std::list<Edge*> EdgeList;
            typedef typename std::list<Edge*>::iterator EdgeListIterator;
        private:
            _SegmentedImage<_width,_height> m_segmentation;
            vtd_framework::utils::_Matrix<_width,_height,_type>* m_data;
            EdgeList m_edges;
            uchar m_edge_c[_width*_height];
            cv::EM m_em;

        public:

            void pushEdge(int x0, int y0, int x1, int y1) {
                Edge* e;
                e = new Edge(&this->m_segmentation, &this->m_data, x0, y0, x1, y1);
                this->m_edge_c[e->i0()]++;
                this->m_edge_c[e->i1()]++;
                this->m_edges.push_back(e);
            }

            _RegionGrow() : m_segmentation(), m_data(nullptr), m_edges() {

                memset(this->m_edge_c,0,_width*_height*sizeof(uchar));
/*
                for(int y = 0; y < _height; y++)
                    for(int x = 0; x < _width; x++) {
                        if(x < _width - 1) {
                            this->pushEdge(x, y, x + 1, y);
                        } if(y < _height - 1) {
                            this->pushEdge(x, y, x, y + 1);
                        } if(x < _width - 1 && y < _height - 1) {
                            //this->pushEdge(x, y, x + 1, y + 1);
                        } if(x > 0 &&  y < _height - 1) {
                            //this->pushEdge(x, y, x - 1, y + 1);
                        }
                    }
*/
                this->reset();
            }

            void reset() override {

                this->m_data = this->template input<0>()->value();
                this->template output<0>()->value(&this->m_segmentation);
            }

            static bool sort(IEdge<_width,_height,_type>* i, IEdge<_width,_height,_type>* j) {
                return i->w() < j->w();
            }

            void calc() override {
                cv::Mat_<_type> in;
                cv::Mat1f std(_height,_width);
                int s = 20;

                int y0_s, y0_e, x0_s, x0_e, a;
                float weight;

                in = this->template input<0>()->value()->mat();

                for(int y0 = 0; y0 < int(_height); y0+=s) {
                    y0_s = s > y0 ? 0 : y0 - s;
                    y0_e = MIN(y0 + s,int(_height)-1);
                    for(int x0 = 0; x0 < int(_width); x0+=s) {
                        weight = 0;
                        x0_s = s > x0 ? 0 : x0 - s;
                        x0_e = MIN(x0 + s,int(_width-1));
                        a = (y0_e - y0_s + 1)*(x0_e - x0_s + 1);
                        for(int y1 = y0_s; y1 <= y0_e; y1++) {
                            for(int x1 = x0_s; x1 <= x0_e; x1++) {
                                if(y1 != y0 && x1 != x0)
                                weight += calcEdgeWeight(in.template at<_type>(y0,x0),in.template at<_type>(y1,x1)) / (a-1);
                            }
                        }
                        for(int y1 = y0_s; y1 <= y0_e; y1++) {
                            for(int x1 = x0_s; x1 <= x0_e; x1++) {
                                if(y1 != y0 && x1 != x0)
                                    std.template at<float>(y1,x1) = (weight);
                            }
                        }
                    }
                }

                cv::normalize(std,std,0,1,cv::NORM_MINMAX);
                cv::namedWindow("DEBUG");
                cv::imshow("DEBUG",std);


                this->template output<0>()->value(&this->m_segmentation);
            }
        };

        template <>
        float calcEdgeWeight<cv::Vec3b>(cv::Vec3b& v0, cv::Vec3b& v1) {
            //return (float) fabsf(v0.val[0]-v1.val[0]);
            return (float) cv::norm(v0[0]-v1[0]);
        }
    }
}
#endif //REGIONGROW_H
