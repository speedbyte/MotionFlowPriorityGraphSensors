//
// Created by geislerd on 28.04.17.
//

#ifndef EYETRIBE_DRAWGAZE_H
#define EYETRIBE_DRAWGAZE_H

#include <unused/eyetribe/EyeballEstimation.h>
#include <unused/gaze/Gaze.h>

namespace vtd_framework {
    namespace eyetribe {

        static void arrowedLine(CV_IN_OUT cv::Mat& img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
                                int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
        {
            const double tipSize = norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow
            cv::line(img, pt1, pt2, color, thickness, line_type, shift);
            const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );
            cv::Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
                    cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
            cv::line(img, p, pt2, color, thickness, line_type, shift);
            p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
            p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
            cv::line(img, p, pt2, color, thickness, line_type, shift);
        }

        template<uint32_t _format>
        class DrawGaze : public vtd_framework::core::Node::
        template Input<
                typename Format<_format>::Image,
                typename Format<_format>::Image,
                RectificationProjection,
                RectificationProjection,
                gaze::Gaze,
                gaze::Gaze
        >::template Output<
                typename Format<_format>::Image,
                typename Format<_format>::Image
                > {
        private:
            typename Format<_format>::Image m_image[2];

        public:
            DrawGaze() {
                this->template input<0>()->name("left image");
                this->template input<1>()->name("right image");
                this->template input<2>()->name("left projection");
                this->template input<3>()->name("right projection");
                this->template input<4>()->name("left gaze");
                this->template input<5>()->name("right gaze");

                this->template output<0>()->name("left image");
                this->template output<0>()->value(&this->m_image[0]);
                this->template output<1>()->name("right image");
                this->template output<1>()->value(&this->m_image[1]);
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

            void projection(RectificationProjection &P, gaze::Gaze& g, cv::Point2f* v2d) {
                const cv::Matx34f p(P.mat());
                cv::Point3f a;
                cv::Vec3f o, d, e;

                o = g.origin();
                d = g.dir();
                e = o + (d * 100);

                a = p*cv::Vec4f(o.val[0],o.val[1],o.val[2],1.0f);
                v2d[0].x = a.x/a.z;
                v2d[0].y = a.y/a.z;

                a = p*cv::Vec4f(e.val[0],e.val[1],e.val[2],1.0f);
                v2d[1].x = a.x/a.z;
                v2d[1].y = a.y/a.z;
            }

            void calc() override {
                cv::Mat1b& in0 = *this->template input<0>()->value();
                cv::Mat1b& in1 = *this->template input<1>()->value();
                RectificationProjection& p0 = *this->template input<2>()->value();
                RectificationProjection& p1 = *this->template input<3>()->value();
                gaze::Gaze& leftGaze = *this->template input<4>()->value();
                gaze::Gaze& rightGaze = *this->template input<5>()->value();
                gaze::Gaze centerGaze((leftGaze.origin()+rightGaze.origin())*0.5f,(leftGaze.dir()+rightGaze.dir())*0.5f);

                cv::Point2f leftImageLeftGaze[2];
                cv::Point2f rightImageLeftGaze[2];
                cv::Point2f leftImageRightGaze[2];
                cv::Point2f rightImageRightGaze[2];
                cv::Point2f leftImageCenterGaze[2];
                cv::Point2f rightImageCenterGaze[2];

                this->projection(p0,leftGaze,leftImageLeftGaze);
                this->projection(p1,leftGaze,rightImageLeftGaze);
                this->projection(p0,rightGaze,leftImageRightGaze);
                this->projection(p1,rightGaze,rightImageRightGaze);
                this->projection(p0,centerGaze,leftImageCenterGaze);
                this->projection(p1,centerGaze,rightImageCenterGaze);

                this->m_image[0].mat(in0);
                this->m_image[1].mat(in1);

                arrowedLine(this->m_image[0],leftImageLeftGaze[0],leftImageLeftGaze[1],cv::Scalar(255.0));
                arrowedLine(this->m_image[0],leftImageRightGaze[0],leftImageRightGaze[1],cv::Scalar(255.0));
                arrowedLine(this->m_image[1],rightImageLeftGaze[0],rightImageLeftGaze[1],cv::Scalar(255.0));
                arrowedLine(this->m_image[1],rightImageRightGaze[0],rightImageRightGaze[1],cv::Scalar(255.0));
                arrowedLine(this->m_image[0],leftImageCenterGaze[0],leftImageCenterGaze[1],cv::Scalar(255.0));
                arrowedLine(this->m_image[1],rightImageCenterGaze[0],rightImageCenterGaze[1],cv::Scalar(255.0));

                cv::namedWindow("LEFT");
                cv::imshow("LEFT",this->m_image[0]);
                cv::namedWindow("RIGHT");
                cv::imshow("RIGHT",this->m_image[1]);
            }

            void reset() override {

            }

        };
    }
}
#endif //EYETRIBE_DRAWGAZE_H
