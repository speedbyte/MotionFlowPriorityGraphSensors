//
// Created by geislerd on 22.03.17.
//

#ifndef RANDOM_H
#define RANDOM_H

#include <core/Node.h>
#include "GazeMovement.h"
#include <boost/random.hpp>

namespace vtd_framework {
    namespace gaze {
        class Random : public vtd_framework::core::Node::
        template Input<>::
        template Output<Gaze,GazeMovement> {
        private:
            Gaze m_gaze;
            GazeMovement m_gm;
            std::vector<GazeMovement> m_gms;
            boost::random::mt19937 m_rng;

        public:
            Random();

            Gaze gaze(const float& inclination_rate, const float& azimuth_rate, const float& x_rate, const float& y_rate, const float& z_rate);

            void calc() override;

            void reset() override;
        };
    }
}

#endif //RANDOM_H
