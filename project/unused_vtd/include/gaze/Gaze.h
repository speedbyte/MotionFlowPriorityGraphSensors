//
// Created by geislerd on 22.03.17.
//

#ifndef GAZE_H
#define GAZE_H

#include <opencv2/opencv.hpp>
#include <core/Node.h>

namespace vtd_framework {
    namespace gaze {
        class Gaze {
        private:
            cv::Vec3f m_origin;
            cv::Vec2f m_angle;

        public:

            enum Component {
                ORIGIN_X = 0,
                ORIGIN_Y = 1,
                ORIGIN_Z = 2,
                ANGLE_INCLINATION = 3,
                ANGLE_AZIMUTH = 4
            };

            template<Component _component>
            class Selector : public vtd_framework::core::Node::
            template Input<Gaze>::
            template Output<float> {
            public:
                Selector();
                void calc() override;
                void reset() override;
            };

            Gaze(cv::Vec3f origin, cv::Vec2f angle);
            Gaze(cv::Point3f origin, cv::Vec2f angle);
            Gaze(cv::Vec3f origin, cv::Vec3f direction);
            Gaze(cv::Point3f origin, cv::Vec3f direction);
            Gaze(float x, float y, float z, float inclination, float azimuth);
            Gaze(float x, float y, float z, float dx, float dy, float dz);

            cv::Vec3f& origin();
            cv::Vec2f& angle();
            cv::Vec3f dir();

            Gaze& operator+=(const Gaze& gaze);

        };
    }
}
#endif //GAZE_H
