//
// Created by geislerd on 19.04.17.
//

#ifndef EYETRIBE_GLINTDETECTOR_H
#define EYETRIBE_GLINTDETECTOR_H

#include <unused/eyetribe/VideoReader.h>

namespace vtd_framework {
    namespace eyetribe {

        typedef std::pair<cv::Point,double> Glint;
        typedef std::vector<Glint> GlintList;

        static int id = 0;

        template<uint32_t _format>
        class GlintDetector : public vtd_framework::core::Node::
        template Input<typename Format<_format>::Image>::template Output<GlintList,cv::Rect> {
        private:
            GlintList m_glints;
            cv::Mat1f m_dog[2];
            cv::Ptr<cv::FilterEngine> m_filter;
            cv::Rect m_roi;

        public:
            GlintDetector() {
                this->template input<0>()->name("image");

                this->template output<0>()->name("glints");
                this->template output<0>()->value(&this->m_glints);

                this->template output<1>()->name("roi");
                this->template output<1>()->value(&this->m_roi);

                this->reset();
            }

#define dshow( name , I ) do {\
            cv::Mat1f name##_tmp_d; \
            cv::normalize(I,name##_tmp_d,0.0f,1.0f,cv::NORM_MINMAX,CV_32FC1); \
            cv::namedWindow(#name); \
            cv::imshow(#name,name##_tmp_d); \
            } while(false);

            float calcSigma(int ksize) {
                return 0.3f*((ksize-1)*0.5f - 1.0f) + 0.8f;
            }

            void glintKernel(const cv::Mat1b& in, cv::Mat1b& out, int s) {
                cv::Mat1f mag, kernel, sub;//(5,7,kernel_data);
                cv::Mat1b mean, grad, rgrad;
                cv::Size ksize(13,13);
                kernel = cv::Mat1f(ksize);
                kernel.setTo(-1.0f);
                kernel.template at<float>((ksize.height-1)/2,(ksize.width-1)/2) = float(-cv::sum(kernel).val[0])-1.0f;

                //for(int i = 0; i < s; i++)
                //    cv::pyrUp(kernel,kernel);

                // normalize kernel
                //cv::divide(kernel,kernel.cols*kernel.rows,kernel);

                // mean image
                cv::blur(in,mean,ksize);

                // mag image
                cv::subtract(in,mean,sub,cv::noArray(),CV_32FC1);
                cv::subtract(sub,cv::Scalar::all(50),sub,cv::noArray(),CV_32FC1);
                mag = cv::Mat1f::zeros(in.rows,in.cols);
                //cv::sepFilter2D(mag,mag,-1,this->m_dog[0],this->m_dog[1]);
                this->m_filter->apply(sub,mag);
                mag.convertTo(out,CV_8UC1);
                //cv::add(mag,cv::mean(mean),out,cv::noArray(),CV_8UC1);

                cv::GaussianBlur(out,out,cv::Size(7,7),1,1);
            }

            void thresh(const cv::Mat1b &in, cv::Mat1b &mag, cv::Mat1b &thresh, int glint_size) {
                double meanVal, maxVal, threshVal;

                this->glintKernel(in,mag,0);

                meanVal = cv::mean(mag).val[0];
                cv::minMaxLoc(mag, nullptr,&maxVal, nullptr, nullptr);
                threshVal = (maxVal-meanVal)*0.5;

                //cv::adaptiveThreshold(mag, thresh,255,cv::ADAPTIVE_THRESH_MEAN_C ,cv::THRESH_BINARY, glint_size,-threshVal);
                cv::threshold(mag,thresh,threshVal,255.0,cv::THRESH_BINARY);
                cv::dilate(thresh,thresh,cv::Mat1b::ones(glint_size,glint_size));
            }

            int m_id = id++;

            void show(const cv::Mat1b& in, const cv::Mat1b& mag, const cv::Mat1b& thres) {
                cv::Mat1b magrs, threshrs;
                cv::Mat3b rgb, rgbmag, rgbthresh;
                std::stringstream name;

                name << "DEBUG " << this->m_id << " ";

                cv::cvtColor(in,rgb,cv::COLOR_GRAY2BGR);
                cv::rectangle(rgb,this->m_roi,cv::Scalar(255,0,0),2);

                cv::namedWindow(name.str() + "ROI");
                cv::imshow(name.str() + "ROI",rgb);

                threshrs = cv::Mat1b::zeros(in.rows,in.cols);
                thres.copyTo(threshrs(this->m_roi));
                cv::applyColorMap(threshrs,rgbthresh,cv::COLORMAP_JET);
                cv::rectangle(rgbthresh,this->m_roi,cv::Scalar(255,0,0),2);

                cv::namedWindow(name.str() + "THRESH");
                cv::imshow(name.str() + "THRESH",rgbthresh);

                magrs = cv::Mat1b::zeros(in.rows,in.cols);
                mag.copyTo(magrs(this->m_roi));
                cv::applyColorMap(magrs,rgbmag,cv::COLORMAP_JET);
                cv::rectangle(rgbmag,this->m_roi,cv::Scalar(255,0,0),2);

                cv::namedWindow(name.str() + "MAG");
                cv::imshow(name.str() + "MAG",rgbmag);
            }

            void calc() override {
                cv::Mat1b in, ins, thresh, mag;
                int sz;
                Glint tmp;

                this->m_glints.clear();

                // make sure we have a valid roi
                this->m_roi &= cv::Rect(0,0,Format<_format>::Image::WIDTH,Format<_format>::Image::HEIGHT);
                if(this->m_roi.area() <= 0)
                    this->m_roi = cv::Rect(0,0,Format<_format>::Image::WIDTH,Format<_format>::Image::HEIGHT);

                in = this->template input<0>()->value()->mat();
                sz = this->properties()->template get<int>("glint_size",51);

                this->thresh(in(this->m_roi), mag, thresh, sz);
                //this->show(in,mag,thresh);

                for(int i = 0; true; i++) {

                    if(i > 5) {
                        this->m_glints.clear();
                        return;
                    }

                    tmp.first = cv::Point(-1,-1);

                    cv::minMaxLoc(mag, nullptr, &tmp.second, nullptr, &tmp.first,thresh);

                    if(tmp.first.x <= 0 || tmp.first.y <= 0)
                        break;

                    cv::floodFill(thresh,tmp.first,0);

                    tmp.first += this->m_roi.tl();

                    this->m_glints.push_back(tmp);
                }
            }

            void reset() override {
                cv::Mat1f hi, lo, dog;

                hi = cv::getGaussianKernel(21,0.1f,CV_32FC1);
                lo = cv::getGaussianKernel(21,5.0f,CV_32FC1);

                cv::subtract(hi,lo,dog,cv::noArray(),CV_32FC1);

                this->m_dog[1] = dog;
                cv::subtract(this->m_dog[1],cv::sum(this->m_dog[1]),this->m_dog[1]);

                this->m_dog[0] = cv::Mat1f(25,1);
                for(int i = 0; i < 25; i++) {
                    this->m_dog[0].template at<float>(i) =
                            dog.template at<float>((i-2)%25)+
                            dog.template at<float>((i-0)%25)+
                            dog.template at<float>((i+2)%25);
                }
                cv::subtract(this->m_dog[0],cv::sum(this->m_dog[0]),this->m_dog[0]);

                this->m_filter = cv::createSeparableLinearFilter(CV_32FC1,CV_32FC1,this->m_dog[0],this->m_dog[1]);

                this->m_roi = cv::Rect(0,0,Format<_format>::Image::WIDTH,Format<_format>::Image::HEIGHT);
            }
        };
    }
}
#endif //EYETRIBE_GLINTDETECTOR_H
