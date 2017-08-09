//
// Created by geislerd on 02.05.17.
//

#ifndef EYETRIBE_PUPILDETECTION_H
#define EYETRIBE_PUPILDETECTION_H

#include <unused/eyetribe/EyeballEstimation.h>
#include <unused/gaze/Gaze.h>
#include "EyeballEstimation.h"
#include "StereoGlintMatcher.h"

namespace vtd_framework {
    namespace eyetribe {

        template<uint32_t _format>
        class PupilDetection : public vtd_framework::core::Node::
        template Input<
                typename Format<_format>::Image,
                typename Format<_format>::Image,
                RectificationProjection,
                RectificationProjection,
                Eyeball
        >::template Output<
                gaze::Gaze,
                float
        > {
        private:
            gaze::Gaze m_gaze;
            float m_confidence;
            cv::Mat1f m_theta;
            cv::Mat1f m_phi;
            cv::Mat1f m_p;

        public:
            PupilDetection() : m_gaze(0.0f,0.0f,0.0f,0.0f,0.0f), m_confidence(0.0f) {
                this->template input<0>()->name("left image");
                this->template input<1>()->name("right image");
                this->template input<2>()->name("left projection");
                this->template input<3>()->name("right projection");
                this->template input<4>()->name("eyeball");

                this->template output<0>()->name("gaze");
                this->template output<0>()->value(&this->m_gaze);
                this->template output<1>()->name("confidence");
                this->template output<1>()->value(&this->m_confidence);

                this->reset();
            }

            void getEyeballBoundingBox(const Eyeball &eb, cv::Point3f bb[2]) {
                bb[0].x = eb.center.x - eb.radius;
                bb[0].y = eb.center.y - eb.radius;
                bb[0].z = eb.center.z - eb.radius;

                bb[1].x = eb.center.x + eb.radius;
                bb[1].y = eb.center.y + eb.radius;
                bb[1].z = eb.center.z + eb.radius;
            }

            void getEyeballBoundingRect(const cv::Point3f bb[2], const RectificationProjection &p, cv::Rect& rect) {
                cv::Matx34f P(p);
                cv::Vec4f v4[2];
                cv::Vec3f v3[2];
                cv::Vec2f v2[2];

                for(int i = 0; i < 2; i++) {
                    v4[i] = cv::Vec4f(bb[i].x,bb[i].y,bb[i].z,1.0f);
                    v3[i] = P * v4[i];
                    v2[i] = cv::Vec2f(v3[i].val[0]/v3[i].val[2],v3[i].val[1]/v3[i].val[2]);
                }

                rect.x = int(floor(MIN(v2[0].val[0],v2[1].val[0])));
                rect.y = int(floor(MIN(v2[0].val[1],v2[1].val[1])));
                rect.width = int(ceil(MAX(v2[0].val[0],v2[1].val[0])))-rect.x;
                rect.height = int(ceil(MAX(v2[0].val[1],v2[1].val[1])))-rect.y;
            }

            void removeGlint(const cv::Mat1b &in, cv::Mat1b& out) {
                cv::Mat1b t, tg, td, inn;
                cv::Point p;
                cv::Scalar m;

                cv::normalize(in,inn,0,255,cv::NORM_MINMAX);
                cv::threshold(inn, t, 64, 255, cv::THRESH_BINARY);

                out = in;

                for(int i = 0; true; i++) {
                    p = cv::Point(-1,-1);
                    cv::minMaxLoc(out, nullptr, nullptr, nullptr, &p, t);
                    if(p.x < 0 || p.y < 0)
                        return;

                    tg = cv::Mat1b::zeros(t.rows+2,t.cols+2);
                    cv::floodFill(t,tg,p,255, nullptr,cv::Scalar(),cv::Scalar(),4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
                    tg = tg(cv::Rect(1,1,t.cols,t.rows));
                    cv::dilate(tg,td,cv::Mat1b::ones(3,3));
                    cv::bitwise_xor(td,tg,td);
                    out.setTo(cv::mean(out,td),tg);
                    t.setTo(cv::Scalar::all(0),tg);
                }


            }

            cv::Point detectPupil(const cv::Mat1b& img) {
                cv::Mat1b imgm,imgn,imgt;
                cv::Mat1f imgf;
                cv::Point p;

                this->removeGlint(img,imgm);

                cv::subtract(cv::Scalar::all(255),imgm,imgm);
                cv::normalize(imgm,imgm,0,255,cv::NORM_MINMAX);

                for(int i = 0; i < 4; i++) {
                    cv::blur(imgm,imgm,cv::Size(5,5));
                    cv::subtract(imgm,cv::Scalar::all(128),imgm);
                    cv::normalize(imgm,imgm,0,255,cv::NORM_MINMAX);
                }

                cv::threshold(imgm, imgt, 128, 255, cv::THRESH_BINARY);

                p = cv::Point(-1,-1);
                cv::minMaxLoc(imgm, nullptr, nullptr, nullptr,&p,imgt);

                return p;
            }

            void triangulatePoints(cv::Point2f p[2], cv::Point3f& pupil, const RectificationProjection& lp, const RectificationProjection& rp) {
                cv::Mat1f left, right, world;
                float scale;

                left = cv::Mat1f(2,1);
                right = cv::Mat1f(2,1);
                world = cv::Mat1f(4,1);

                left.template at<float>(0,0) = p[0].x;
                left.template at<float>(1,0) = p[0].y;
                right.template at<float>(0,0) = p[1].x;
                right.template at<float>(1,0) = p[1].y;

                cv::triangulatePoints(lp,rp,left,right,world);

                scale = world.template at<float>(3,0);
                pupil.x = world.template at<float>(0,0) / scale;
                pupil.y = world.template at<float>(1,0) / scale;
                pupil.z = world.template at<float>(2,0) / scale;
            }

            void spherePoints(const Eyeball &eb, cv::Mat3f &points) {
                cv::Vec3f* t;
                float sintheta, costheta, sinphi, cosphi;

                points.create(this->m_theta.rows,this->m_phi.cols);

                for (int x = 0; x < this->m_phi.cols; x++) {
                    for(int y = 0; y < this->m_theta.rows; y++) {
                        sinphi = sinf(this->m_phi.template at<float>(y, x));
                        cosphi = cosf(this->m_phi.template at<float>(y, x));
                        sintheta = sinf(this->m_theta.template at<float>(y, x));
                        costheta = cosf(this->m_theta.template at<float>(y, x));

                        t = &points.template at<cv::Vec3f>(y, x);
                        t->val[0] = eb.center.x + eb.radius * sintheta * cosphi;
                        t->val[1] = eb.center.y + eb.radius * sintheta * sinphi;
                        t->val[2] = eb.center.z + eb.radius * costheta;
                    }
                }

            }

            void projectPoints(const RectificationProjection &P, const cv::Mat3f& xyz, cv::Mat2f& xy) {
                const cv::Vec3f* t;
                cv::Vec3f h;

                xy.create(xyz.rows,xyz.cols);
                for(int y = 0; y < xyz.rows; y++) {
                    for(int x = 0; x < xyz.cols; x++) {
                        t = &xyz.template at<cv::Vec3f>(y, x);
                        h = cv::Matx34f(P) * cv::Vec4f(t->val[0],t->val[1],t->val[2],1.0f);
                        xy.template at<cv::Vec2f>(y,x) =cv::Vec2f(h.val[0]/h.val[2],h.val[1]/h.val[2]);
                    }
                }
            }

            void angleP(const float &theta, const float &phi, const cv::Vec2f &xy, const cv::Mat1b &img, float &p) {
                float tp;
                if(xy.val[0] < 0 || xy.val[0] >= Format<_format>::WIDTH)
                    return;
                if(xy.val[1] < 0 || xy.val[1] >= Format<_format>::HEIGHT)
                    return;

                tp = 1.0f/float(img.template at<uchar>(xy.val[1],xy.val[0]));

                if(p != 0.0f)
                    p = 0.5f*p + 0.5f*tp;
                else
                    p = tp;
            }

            void angleP(const cv::Mat1b &mat, const RectificationProjection &P, const Eyeball &eb, cv::Mat1f &p) {
                cv::Mat3f xyz;
                cv::Mat2f xy;

                this->spherePoints(eb,xyz);
                this->projectPoints(P,xyz,xy);

                p.create(this->m_theta.rows,this->m_phi.cols);
                p.setTo(0.0f);
                for(int y = 0; y < this->m_theta.rows; y++) {
                    for (int x = 0; x < this->m_phi.cols; x++) {
                        this->angleP(
                                this->m_theta.template at<float>(y,x),
                                this->m_phi.template at<float>(y,x),
                                xy.template at<cv::Vec2f>(y,x),
                                mat,
                                p.template at<float>(y,x));
                    }
                }
            }

            void angleFP(const cv::Mat1f& raw, cv::Mat1f& f) {
                cv::Mat1f raw01;
                const int ksize = 21;
                const float sigma = 0.3f*((ksize-1.0f)*0.5f-1.0f)+0.8f;

                cv::flip(raw,raw01,0);
                cv::hconcat(raw01,raw,f);
                cv::hconcat(f,raw01,f);
                cv::flip(f,raw01,0);
                cv::vconcat(raw01,f,f);
                cv::vconcat(f,raw01,f);

                cv::GaussianBlur(f,f,cv::Size(ksize,ksize),sigma,sigma,cv::BORDER_CONSTANT);

                f = f(cv::Rect(raw.cols,raw.rows,raw.cols,raw.rows));
            }

            void calc() override {
                const cv::Mat1b& limg = *this->template input<0>()->value();
                const cv::Mat1b& rimg = *this->template input<1>()->value();
                const RectificationProjection& lp = *this->template input<2>()->value();
                const RectificationProjection& rp = *this->template input<3>()->value();
                const Eyeball& eb = *this->template input<4>()->value();
                cv::Point angle;
                float n;
                double confidence;

                cv::Mat1f leftp, rightp, leftfp, rightfp, combp;

                this->m_confidence = 0;
                if(eb.glint.confidence <= 0)
                    return;

                this->angleP(limg,lp,eb,leftp);
                this->angleP(rimg,rp,eb,rightp);

                this->angleFP(leftp,leftfp);
                this->angleFP(rightp,rightfp);

                cv::multiply(leftfp,leftfp,combp);
                cv::divide(combp,cv::sum(combp),combp);

                cv::minMaxLoc(combp, nullptr, &confidence, nullptr, &angle);

                cv::addWeighted(this->m_p,0.75,combp,0.25,0.0,this->m_p);

                cv::minMaxLoc(this->m_p, nullptr, &confidence, nullptr, &angle);

                this->m_confidence = float(confidence);
                this->m_gaze = gaze::Gaze(eb.center,cv::Vec2f(
                        this->m_theta.template at<float>(angle),
                        this->m_phi.template at<float>(angle)));
            }

            void reset() override {
                const int n_theta = 100;
                const int n_phi = 100;
                float theta, phi;

                this->m_theta.create(n_theta,1);
                this->m_phi.create(1,n_phi);

                for(int i = 0; i < n_theta; i++)
                    this->m_theta.template at<float>(i) = (float(i+1)/float(n_theta)) * float(M_PI) + float(M_PI_2);

                for(int i = 0; i < n_phi; i++)
                    this->m_phi.template at<float>(i) = (float(i+1)/float(n_phi)) * float(M_PI);

                cv::repeat(this->m_theta,1,n_phi,this->m_theta);
                cv::repeat(this->m_phi,n_theta,1,this->m_phi);

                this->m_p = cv::Mat1f::ones(n_theta,n_phi);
                cv::divide(this->m_p,cv::sum(this->m_p),this->m_p);
            }
        };
    }
}

#endif //EYETRIBE_PUPILDETECTION_H
