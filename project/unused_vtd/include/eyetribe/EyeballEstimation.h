//
// Created by geislerd on 28.04.17.
//

#ifndef EYETRIBE_EYEBALLESTIMATION_H
#define EYETRIBE_EYEBALLESTIMATION_H

#include <unused/eyetribe/StereoGlintMatcher.h>

namespace vtd_framework {
    namespace eyetribe {

        typedef struct {
            cv::Point3f center;
            float radius;
            StereoGlint glint;
        } Eyeball;

        class EyeballEstimation : public vtd_framework::core::Node::
        template Input<
                StereoGlint,
                StereoGlint
        >::template Output<
                Eyeball,
                Eyeball
                > {
        private:
            Eyeball m_eyeball[2];
        public:
            EyeballEstimation() {
                this->template input<0>()->name("left");
                this->template input<1>()->name("right");

                this->template output<0>()->name("left");
                this->template output<0>()->value(&this->m_eyeball[0]);

                this->template output<1>()->name("right");
                this->template output<1>()->value(&this->m_eyeball[1]);

                this->reset();
            }

            Eyeball eyeball(const StereoGlint& glint, const float& radius) {
                Eyeball eyeball;
                cv::Vec3f dir;

                dir = glint.world;
                dir /= cv::norm(dir);

                eyeball.center = glint.world + cv::Point3f(dir*radius);
                eyeball.glint = glint;
                eyeball.radius = radius;

                return eyeball;
            }

            void calc() override {
                const StereoGlint& leftGlint = *this->template input<0>()->value();
                const StereoGlint& rightGlint = *this->template input<1>()->value();
                const float& radius = this->properties()->template get<float>("radius",12);

                this->m_eyeball[0] = this->eyeball(leftGlint,radius);
                this->m_eyeball[1] = this->eyeball(rightGlint,radius);
            }

            void reset() override {
            }

        };
    }
}
#endif //EYETRIBE_EYEBALLESTIMATION_H
