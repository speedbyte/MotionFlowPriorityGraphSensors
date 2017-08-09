//
// Created by geislerd on 28.04.17.
//

#ifndef EYETRIBE_DRAWEYEBAL_H
#define EYETRIBE_DRAWEYEBAL_H

#include <unused/eyetribe/EyeballEstimation.h>
#include "EyeballEstimation.h"

namespace vtd_framework {
    namespace eyetribe {

        template<uint32_t _format>
        class DrawEyeball : public vtd_framework::core::Node::
        template Input<
                typename Format<_format>::Image,
                typename Format<_format>::Image,
                RectificationProjection,
                RectificationProjection,
                Eyeball,
                Eyeball
        >::template Output<
                typename Format<_format>::Image,
                typename Format<_format>::Image
                > {
        private:
            typename Format<_format>::Image m_image[2];

        public:
            DrawEyeball() {
                this->template input<0>()->name("left image");
                this->template input<1>()->name("right image");
                this->template input<2>()->name("left projection");
                this->template input<3>()->name("right projection");
                this->template input<4>()->name("left eyeball");
                this->template input<5>()->name("right eyeball");

                this->template output<0>()->name("left image");
                this->template output<0>()->value(&this->m_image[0]);
                this->template output<1>()->name("right image");
                this->template output<1>()->value(&this->m_image[1]);
            }

            void spherePoints(Eyeball eyeball, std::vector<cv::Point3f>& points, std::vector<float>& vtheta, std::vector<float>& vphi, int steps) {
                const float step = float(M_PI) / steps;
                cv::Point3f p;

                for(float theta = step/4.0f; theta <= float(M_PI_2) + step / 2.0f; theta+=step) {
                    for (float phi = step/4.0f; phi <= 2.0f * float(M_PI) + step / 2.0f; phi += step) {
                        p.x = eyeball.radius * sinf(theta) * cosf(phi);
                        p.y = eyeball.radius * sinf(theta) * sinf(phi);
                        p.z = eyeball.radius * cosf(theta);
                        p+=eyeball.center;
                        points.push_back(p);
                        vtheta.push_back(theta);
                        vphi.push_back(phi);
                    }
                }
            }

            void drawSphere(typename Format<_format>::Image& img, const std::vector<cv::Point2f>& points, const float& confidence) {
                size_t sz;
                cv::Point2f p[2];
                cv::Scalar color;

                sz = points.size();
                color = cv::Scalar::all(confidence < FLT_EPSILON ? 128.0 : 255.0);

                for(size_t i = 1; i < sz; i++) {
                    p[0] = points[i-1];
                    p[1] = points[i];
                    cv::line(img,p[0],p[1],color);
                }
            }

            void projection(RectificationProjection &P, std::vector<cv::Point3f> &v3d, std::vector<cv::Point2f> &v2d) {
                const cv::Matx34f p(P.mat());
                cv::Point3f a;

                for(std::vector<cv::Point3f>::iterator i = v3d.begin(); i != v3d.end(); ++i) {
                    a = p*cv::Vec4f(i->x,i->y,i->z,1.0);
                    v2d.push_back(cv::Point2f(a.x/a.z,a.y/a.z));
                }
            }

            void calc() override {
                cv::Mat1b& in0 = *this->template input<0>()->value();
                cv::Mat1b& in1 = *this->template input<1>()->value();
                RectificationProjection& p0 = *this->template input<2>()->value();
                RectificationProjection& p1 = *this->template input<3>()->value();
                Eyeball& leftEyeball = *this->template input<4>()->value();
                Eyeball& rightEyeball = *this->template input<5>()->value();

                const int& steps = this->properties()->template get<int>("steps",50);

                std::vector<float> theta, phi;
                std::vector<cv::Point3f> leftSphere3D, rightSphere3D;
                std::vector<cv::Point2f> leftSphere2D[2], rightSphere2D[2];

                this->spherePoints(leftEyeball,leftSphere3D,theta,phi,steps);
                this->spherePoints(rightEyeball,rightSphere3D,theta,phi,steps);

                this->projection(p0,leftSphere3D,leftSphere2D[0]);
                this->projection(p1,leftSphere3D,leftSphere2D[1]);
                this->projection(p0,rightSphere3D,rightSphere2D[0]);
                this->projection(p1,rightSphere3D,rightSphere2D[1]);

                this->m_image[0].mat(in0);
                this->m_image[1].mat(in1);

                this->drawSphere(this->m_image[0],leftSphere2D[0] ,leftEyeball.glint.confidence);
                this->drawSphere(this->m_image[0],rightSphere2D[0],rightEyeball.glint.confidence);
                this->drawSphere(this->m_image[1],leftSphere2D[1] ,leftEyeball.glint.confidence);
                this->drawSphere(this->m_image[1],rightSphere2D[1],rightEyeball.glint.confidence);
            }

            void reset() override {

            }

        };
    }
}
#endif //EYETRIBE_DRAWEYEBAL_H
