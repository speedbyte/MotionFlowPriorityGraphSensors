//
// Created by geislerd on 29.04.17.
//

#ifndef EYETRIBE_STEREOGLINTFILTER_H
#define EYETRIBE_STEREOGLINTFILTER_H

#include <unused/eyetribe/StereoGlintMatcher.h>
#include "StereoGlintMatcher.h"

namespace vtd_framework {
    namespace eyetribe {
        class StereoGlintFilter : public vtd_framework::core::Node::
        template Input<
                StereoGlint,
                StereoGlint
        >::template Output<
                StereoGlint,
                StereoGlint
        > {
        private:
            cv::KalmanFilter m_kf;

            float m_transitionMatrix[6*3*6*3] {
                    //x    y     z     ox    oy    oz    dx    dy    dz    dox   doy   doz   ddx   ddy   ddz  ddox  ddoy  ddoz
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

            float m_measurementNoiseCov[6*6] {
                    //x    y     z     ox    oy    oz    dx    dy    dz    dox   doy   doz   ddx   ddy   ddz  ddox  ddoy  ddoz
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

            float m_measurementMatrix[6*3*6] {
                    //x    y     z     ox    oy    oz    dx    dy    dz    dox   doy   doz   ddx   ddy   ddz  ddox  ddoy  ddoz
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

            float m_processNoiseCov[6*3*6*3] {
                    //x    y     z     ox    oy    oz    dx    dy    dz    dox   doy   doz   ddx   ddy   ddz  ddox  ddoy  ddoz
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f};

            float m_errorCovPost[6*3*6*3] {
                    //x    y     z     ox    oy    oz    dx    dy    dz    dox   doy   doz   ddx   ddy   ddz  ddox  ddoy  ddoz
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f};

            bool m_kalman_initialized;

            StereoGlint m_glint[2];

        public:
            StereoGlintFilter() {
                this->template input<0>()->name("left");
                this->template input<1>()->name("right");

                this->template output<0>()->name("left");
                this->template output<0>()->value(&this->m_glint[0]);

                this->template output<1>()->name("right");
                this->template output<1>()->value(&this->m_glint[1]);

                this->reset();
            }

            cv::Vec6f codeGlints(StereoGlint glint[2]) {
                cv::Vec6f v;

                v.val[0] = (glint[0].world.x + glint[1].world.x) / 2.0f; //
                v.val[1] = (glint[0].world.y + glint[1].world.y) / 2.0f; // center of both glints
                v.val[2] = (glint[0].world.z + glint[1].world.z) / 2.0f; //

                v.val[3] = v.val[0] - glint[0].world.x; //
                v.val[4] = v.val[1] - glint[0].world.y; // translation to left glint
                v.val[5] = v.val[2] - glint[0].world.z; //

                return v;
            }

            void decodeGlints(const cv::Vec6f& v) {
                this->m_glint[0].world.x = v.val[0] - v.val[3];
                this->m_glint[0].world.y = v.val[1] - v.val[4];
                this->m_glint[0].world.z = v.val[2] - v.val[5];
                this->m_glint[1].world.x = v.val[0] + v.val[3];
                this->m_glint[1].world.y = v.val[1] + v.val[4];
                this->m_glint[1].world.z = v.val[2] + v.val[5];
            }

            void initKalman(StereoGlint glint[2]) {
                const cv::Vec6f v = this->codeGlints(glint);

                this->m_kf.statePre.template at<float>(0)  = v.val[0];
                this->m_kf.statePre.template at<float>(1)  = v.val[1];
                this->m_kf.statePre.template at<float>(2)  = v.val[2];
                this->m_kf.statePre.template at<float>(3)  = v.val[3];
                this->m_kf.statePre.template at<float>(4)  = v.val[4];
                this->m_kf.statePre.template at<float>(5)  = v.val[5];

                this->m_kf.statePre.template at<float>(6)  = 0.0f;
                this->m_kf.statePre.template at<float>(7)  = 0.0f;
                this->m_kf.statePre.template at<float>(8)  = 0.0f;
                this->m_kf.statePre.template at<float>(9)  = 0.0f;
                this->m_kf.statePre.template at<float>(10) = 0.0f;
                this->m_kf.statePre.template at<float>(11) = 0.0f;

                this->m_kf.statePre.template at<float>(12) = 0.0f;
                this->m_kf.statePre.template at<float>(13) = 0.0f;
                this->m_kf.statePre.template at<float>(14) = 0.0f;
                this->m_kf.statePre.template at<float>(15) = 0.0f;
                this->m_kf.statePre.template at<float>(16) = 0.0f;
                this->m_kf.statePre.template at<float>(17) = 0.0f;

                this->m_kf.statePost.template at<float>(0)  = v.val[0];
                this->m_kf.statePost.template at<float>(1)  = v.val[1];
                this->m_kf.statePost.template at<float>(2)  = v.val[2];
                this->m_kf.statePost.template at<float>(3)  = v.val[3];
                this->m_kf.statePost.template at<float>(4)  = v.val[4];
                this->m_kf.statePost.template at<float>(5)  = v.val[5];

                this->m_kf.statePost.template at<float>(6)  = 0.0f;
                this->m_kf.statePost.template at<float>(7)  = 0.0f;
                this->m_kf.statePost.template at<float>(8)  = 0.0f;
                this->m_kf.statePost.template at<float>(9)  = 0.0f;
                this->m_kf.statePost.template at<float>(10) = 0.0f;
                this->m_kf.statePost.template at<float>(11) = 0.0f;

                this->m_kf.statePost.template at<float>(12) = 0.0f;
                this->m_kf.statePost.template at<float>(13) = 0.0f;
                this->m_kf.statePost.template at<float>(14) = 0.0f;
                this->m_kf.statePost.template at<float>(15) = 0.0f;
                this->m_kf.statePost.template at<float>(16) = 0.0f;
                this->m_kf.statePost.template at<float>(17) = 0.0f;

                this->m_kalman_initialized = true;
            }

            void correct(StereoGlint glint[2]) {
                cv::Vec6f v = this->codeGlints(glint);
                cv::Mat1f corrected, measurement;

                corrected = this->m_kf.predict(); //update internal states

                measurement = cv::Mat1f(6,1);
                measurement.template at<float>(0) = v.val[0];
                measurement.template at<float>(1) = v.val[1];
                measurement.template at<float>(2) = v.val[2];
                measurement.template at<float>(3) = v.val[3];
                measurement.template at<float>(4) = v.val[4];
                measurement.template at<float>(5) = v.val[5];

                corrected = this->m_kf.correct(measurement);

                v.val[0] = corrected.template at<float>(0);
                v.val[1] = corrected.template at<float>(1);
                v.val[2] = corrected.template at<float>(2);
                v.val[3] = corrected.template at<float>(3);
                v.val[4] = corrected.template at<float>(4);
                v.val[5] = corrected.template at<float>(5);
                this->decodeGlints(v);
            }

            void updateMeasurementError(float confidence) {
                cv::setIdentity(this->m_kf.measurementNoiseCov,cv::Scalar(0.01f/(confidence*confidence)));
            }

            void calc() override {
                StereoGlint glint[2];
                float var;

                glint[0] = *this->template input<0>()->value();
                glint[1] = *this->template input<1>()->value();

                this->m_glint[0] = glint[0];
                this->m_glint[1] = glint[1];

                if(!this->m_kalman_initialized) {
                    if(glint[0].confidence > 0.0f && glint[1].confidence > 0.0f)
                        this->initKalman(glint);
                    return;
                }

                var = MAX(MIN(glint[0].confidence,glint[1].confidence),0.00001f);

                this->updateMeasurementError(var);
                this->correct(glint);
            }

            void reset() override {
                this->m_kalman_initialized = false;

                this->m_kf.init(6*3,6,0,CV_32FC1);
                this->m_kf.transitionMatrix    = cv::Mat1f(6*3,6*3,this->m_transitionMatrix);
                this->m_kf.measurementNoiseCov = cv::Mat1f(6,6,this->m_measurementNoiseCov);
                this->m_kf.measurementMatrix   = cv::Mat1f(6,18,this->m_measurementMatrix);
                this->m_kf.processNoiseCov     = cv::Mat1f(6*3,6*3,this->m_processNoiseCov);
                this->m_kf.errorCovPost        = cv::Mat1f(6*3,6*3,this->m_errorCovPost);
            }
        };
    }
}
#endif //EYETRIBE_STEREOGLINTFILTER_H
