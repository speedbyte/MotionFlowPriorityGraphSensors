//
// Created by geislerd on 19.04.17.
//

#ifndef EYETRIBE_STEREOGLINTMATCHER_H
#define EYETRIBE_STEREOGLINTMATCHER_H

#include <unused/eyetribe/GlintDetector.h>
#include <unused/eyetribe/StereoRectification.h>

namespace vtd_framework {
    namespace eyetribe {

        typedef std::pair<int,Glint> GlintSort;

        typedef struct {
            Glint left;
            Glint right;
            cv::Point3f world;
            float confidence;
        } StereoGlint;
        typedef std::vector<StereoGlint> StereoGlintList;

        class StereoGlintMatcher : public vtd_framework::core::Node::
        template Input<
                GlintList,
                cv::Rect,
                GlintList,
                cv::Rect,
                RectificationProjection,
                RectificationProjection
        >::template Output<
                StereoGlint,
                StereoGlint
        > {
        private:
            StereoGlint m_glints[2];

        public:
            StereoGlintMatcher() {
                this->template input<0>()->name("glints0");
                this->template input<1>()->name("glints1");
                this->template input<2>()->name("P0");
                this->template input<3>()->name("P1");

                this->template output<0>()->name("glints0");
                this->template output<0>()->value(&this->m_glints[0]);
                this->template output<1>()->name("glints1");
                this->template output<1>()->value(&this->m_glints[1]);

                this->reset();
            }

            void triangulatePoints(StereoGlintList &glints) {
                const RectificationProjection& p0 = *this->template input<4>()->value();
                const RectificationProjection& p1 = *this->template input<5>()->value();
                const size_t glints_size = glints.size();
                cv::Mat1f left, right;
                cv::Mat1f world;
                float scale;

                left = cv::Mat1f(2,int(glints_size));
                right = cv::Mat1f(2,int(glints_size));
                world = cv::Mat1f(4,int(glints_size));

                for(int i = 0; i < glints_size; i++) {
                    left.template at<float>(0,i) = glints[i].left.first.x;
                    left.template at<float>(1,i) = glints[i].left.first.y;
                    right.template at<float>(0,i) = glints[i].right.first.x;
                    right.template at<float>(1,i) = glints[i].right.first.y;
                }

                cv::triangulatePoints(p0,p1,left,right,world);

                for(int i = 0; i < glints_size; i++) {
                    scale = world.template at<float>(3,i);
                    glints[i].world.x = world.template at<float>(0,i) / scale;
                    glints[i].world.y = world.template at<float>(1,i) / scale;
                    glints[i].world.z = world.template at<float>(2,i) / scale;
                }
            }

            void triangulatePoints(GlintList &left, GlintList &right, StereoGlintList& glints) {
                const size_t left_size = left.size();
                const size_t right_size = right.size();
                StereoGlint tmp;

                for(GlintList::iterator i = left.begin(); i != left.end(); ++i)
                    for(GlintList::iterator j = right.begin(); j != right.end(); ++j)
                        glints.push_back(StereoGlint {*i,*j,cv::Point3f(),0.0f});

                this->triangulatePoints(glints);
            }

            float pdf(float d, float mean, float sigma) {
                const float s = 1.0f/sqrtf(2.0f*float(M_PI)*sigma*sigma);
                const float ee = (d-mean)*(d-mean)/(2.0f*sigma*sigma);
                return s*expf(-ee);
            }

            float pdf(float d, float mean, float sigma, float confidence) {
                const float lb = (1.0f-confidence)/2.0f;
                const float ub = 1.0f-lb;
                const float pdf = this->pdf(d,mean,sigma)/this->pdf(mean,mean,sigma);
                if(confidence < pdf)
                    return pdf;
                else
                    return 0.0f;
            }

            float pdfTrack(cv::Point3f world[2], float mean, float sigma, float confidence) {
                float d, p, c;
                cv::Point3f m[2];

                m[0] = (world[0]-world[1]) * 0.5f;
                m[1] = (this->m_glints[0].world-this->m_glints[1].world) * 0.5f;
                d = float(cv::norm(m[0]-m[1]));
                p = this->pdf(d,0,sigma,confidence);
                c = this->m_glints[0].confidence;

                return (1.0f-c)+(c*p);
            }

            cv::Mat1f pdist(StereoGlintList glints) {
                const float sigma = this->properties()->template get<float>("sigma",20.0f);
                const float mean = this->properties()->template get<float>("mean",64.0f);
                const float confidence = this->properties()->template get<float>("confidence",0.10f);
                cv::Mat1f dist;
                size_t sz;
                float p, d;
                cv::Point3f w[2];

                dist = cv::Mat1f((int)glints.size(),(int)glints.size());
                sz = glints.size();
                dist.setTo(0.0f);

                for(int i = 0; i < sz; i++)
                    for(int j = i+1; j < sz; j++) {
                        w[0] = glints[i].world;
                        w[1] = glints[j].world;
                        d = float(cv::norm(w[0]-w[1]));
                        p = 1.0f;
                        p *= this->pdf(fabsf(w[0].x-w[1].x),mean,sigma*2.0f,confidence);
                        p *= this->pdf(fabsf(w[0].y-w[1].y),0.0f,sigma*2.0f,confidence);
                        p *= this->pdf(fabsf(w[0].z-w[1].z),0.0f,sigma*2.0f,confidence);
                        p *= this->pdfTrack(w,0.0f,sigma*2.0f,confidence);
                        p *= this->pdf(d,mean,sigma,confidence);
                        if(confidence < p)
                            dist.template at<float>(i,j) = p;
                        else
                            dist.template at<float>(i,j) = 0.0f;

                    }

                return dist;
            }

            StereoGlintList checkRange(StereoGlintList in) {
                StereoGlintList out;
                float dist;
                const float min_dist = this->properties()->template get<float>("min_dist",300.0f);
                const float max_dist = this->properties()->template get<float>("max_dist",1500.0f);
                cv::Point3f p;

                for(StereoGlintList::iterator i = in.begin(); i != in.end(); i++) {
                    p = i->world;
                    dist = float(cv::norm(p));

                    if(p.z > 0.0f && dist > min_dist && max_dist > dist)
                        out.push_back(*i);
                }

                return out;
            }

            float nCr(size_t n, size_t r) {
                float a;

                a = 1.0f;

                if(n>r)
                    for(int j = 1; j <= r; j++)
                        a *= float(n+1-j)/float(j);
            }

            void updateRoi(cv::Rect &roi, const cv::Point& p0, const cv::Point& p1, const float& confidence) {
                cv::Size sz;
                float s;

                if(confidence > 0.0f) {
                    roi = cv::Rect(MIN(p0.x, p1.x),MIN(p0.y,p1.y),abs(p0.x-p1.x),abs(p0.y-p1.y));
                    s = 100.0f/sqrtf(confidence);
                    roi.x -= s;
                    roi.y -= s;
                    roi.width += 2.0f*s;
                    roi.height += 2.0f*s;
                } else
                    roi = cv::Rect(0,0,-1,-1);
            }

            void updateRoi() {
                cv::Rect& roi0 = *this->template input<1>()->value();
                cv::Rect& roi1 = *this->template input<3>()->value();

                this->updateRoi(
                        roi0,
                        this->m_glints[0].left.first,
                        this->m_glints[1].left.first,
                        this->m_glints[0].confidence);
                this->updateRoi(
                        roi1,
                        this->m_glints[0].right.first,
                        this->m_glints[1].right.first,
                        this->m_glints[1].confidence);
            }

            void calc() override {
                GlintList& glints0 = *this->template input<0>()->value();
                GlintList& glints1 = *this->template input<2>()->value();
                cv::Mat1f dist;
                cv::Point max;
                double maxVal;
                StereoGlintList stereoGlints;
                // stop if not enough glints were detected
                if(glints0.size() < 2 || glints1.size() < 2) {

                    // reset confidence to zero
                    this->m_glints[0].confidence = 0.0f;
                    this->m_glints[1].confidence = 0.0f;

                    this->updateRoi();
                    return;
                }

                // triangulate glints
                this->triangulatePoints(glints0,glints1,stereoGlints);

                // dismiss glints outside the headbox
                stereoGlints = this->checkRange(stereoGlints);

                // calculate probabilistic distance metric
                dist = this->pdist(stereoGlints);

                // select most probable glint pair
                maxVal = -1.0; max = cv::Point(-1,-1);
                cv::minMaxLoc(dist, nullptr, &maxVal, nullptr, &max);
                if(maxVal < 0.0 || max.x < 0.0 || max.y < 0.0) {

                    // reset confidence to zero
                    this->m_glints[0].confidence = 0.0f;
                    this->m_glints[1].confidence = 0.0f;

                    this->updateRoi();
                    return;
                }

                // order glints align x axis
                if(stereoGlints[max.x].world.x > stereoGlints[max.y].world.x)
                    std::swap(max.x,max.y);

                this->m_glints[0] = stereoGlints[max.x];
                this->m_glints[1] = stereoGlints[max.y];

                // a-priori to select the right glints as confidence
                this->m_glints[0].confidence = 1.0f/this->nCr(stereoGlints.size(),4);
                // probability from the gaussian model
                this->m_glints[0].confidence *= float(maxVal);
                this->m_glints[1].confidence = this->m_glints[0].confidence;

                //std::cout << "stereo glints: " << stereoGlints.size() << std::endl;
                std::cout << "confidence: " << this->m_glints[0].confidence << std::endl;

                this->updateRoi();

                //std::cout << "left glint: " << this->m_glints[0].world << std::endl;
                //std::cout << "right glint: " << this->m_glints[1].world << std::endl;
            }

            void reset() override {
            }
        };
    }
}

#endif //EYETRIBE_STEREOGLINTMATCHER_H
