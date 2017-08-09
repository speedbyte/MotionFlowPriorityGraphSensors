//
// Created by geislerd on 19.04.17.
//

#ifndef EYETRIBE_STEREORECTIFICATION_H
#define EYETRIBE_STEREORECTIFICATION_H

#include <eyetribe/StereoCalibration.h>

namespace vtd_framework {
    namespace eyetribe {

        typedef vtd_framework::utils::_Matrix<3,3,float> RectificationRotation;
        typedef vtd_framework::utils::_Matrix<4,3,float> RectificationProjection;
        typedef vtd_framework::utils::_Matrix<4,4,float> RectificationDisparity;

        template<uint32_t _format>
        class StereoRectification : public vtd_framework::core::Node::
        template Input<
                CameraMatrix,              // camera 0 matrix
                CameraDistortion,          // camera 0 distortion
                CameraMatrix,              // camera 1 matrix
                CameraDistortion,          // camera 1 distortion
                CameraCameraRotation,      // camera 0 to camera 1 rotation
                CameraCameraTranslation,   // camera 0 to camera 1 translation
                bool                       // is calibrated
        >::template Output<
                RectificationRotation,     // rectification transform (rotation matrix) for the first camera
                RectificationRotation,     // rectification transform (rotation matrix) for the second camera
                RectificationProjection,   // projection matrix in the new (rectified) coordinate systems for the first camera
                RectificationProjection,   // projection matrix in the new (rectified) coordinate systems for the second camera
                RectificationDisparity,    // disparity-to-depth mapping matrix
                bool                       // rectification was successful
        > {
        private:
            RectificationRotation m_R[2];
            RectificationProjection m_P[2];
            RectificationDisparity m_Q;
            bool m_valid;
        public:
            StereoRectification() {
                this->template input<0>()->name("matrix0");
                this->template input<1>()->name("distortion0");
                this->template input<2>()->name("matrix1");
                this->template input<3>()->name("distortion1");
                this->template input<4>()->name("rotation");
                this->template input<5>()->name("translation");
                this->template input<6>()->name("valid");

                this->template output<0>()->name("R0");
                this->template output<0>()->value(&this->m_R[0]);
                this->template output<1>()->name("R1");
                this->template output<1>()->value(&this->m_R[1]);
                this->template output<2>()->name("P0");
                this->template output<2>()->value(&this->m_P[0]);
                this->template output<3>()->name("P1");
                this->template output<3>()->value(&this->m_P[1]);
                this->template output<4>()->name("Q");
                this->template output<4>()->value(&this->m_Q);
                this->template output<5>()->name("valid");
                this->template output<5>()->value(&this->m_valid);

                this->reset();
            }

            void calc() override {
                cv::Mat tmp_R[3], tmp_P[2], tmp_Q, tmp_M[2], tmp_D[2], tmp_T;
                const CameraMatrix& cameraMatrix0           = *this->template input<0>()->value();
                const CameraDistortion& cameraDistortion0   = *this->template input<1>()->value();
                const CameraMatrix& cameraMatrix1           = *this->template input<2>()->value();
                const CameraDistortion& cameraDistortion1   = *this->template input<3>()->value();
                const CameraCameraRotation& R               = *this->template input<4>()->value();
                const CameraCameraTranslation& T            = *this->template input<5>()->value();
                const bool valid                            = *this->template input<6>()->value();
                const cv::Size sz                           = cv::Size(Format<_format>::WIDTH,Format<_format>::HEIGHT);

                if(!valid)
                    return;

                if(this->m_valid)
                    return;

                cameraMatrix0.convertTo(tmp_M[0],CV_64FC1);
                cameraMatrix1.convertTo(tmp_M[1],CV_64FC1);
                cameraDistortion0.convertTo(tmp_D[0],CV_64FC1);
                cameraDistortion1.convertTo(tmp_D[1],CV_64FC1);
                R.convertTo(tmp_R[2],CV_64FC1);
                T.convertTo(tmp_T,CV_64FC1);

                cv::Rect r0, r1;
                cv::stereoRectify(
                        tmp_M[0],
                        tmp_D[0],
                        tmp_M[1],
                        tmp_D[1],
                        sz,
                        tmp_R[2],
                        tmp_T,
                        tmp_R[0],
                        tmp_R[1],
                        tmp_P[0],
                        tmp_P[1],
                        tmp_Q,
                        0,-1.0,cv::Size(),&r0,&r1);

                tmp_P[1].template at<double>(7) = -40.0*1941.3335; //TODO: BAD FIX

                tmp_R[0].convertTo(this->m_R[0],CV_32FC1);
                tmp_R[1].convertTo(this->m_R[1],CV_32FC1);
                tmp_P[0].convertTo(this->m_P[0],CV_32FC1);
                tmp_P[1].convertTo(this->m_P[1],CV_32FC1);
                tmp_Q.convertTo(this->m_Q,CV_32FC1);

                std::cout << "Stereo Rectification: " << std::endl;
                std::cout << "R[0]: " << this->m_R[0] << std::endl;
                std::cout << "P[0]: " << this->m_P[0] << std::endl;
                std::cout << "R[1]: " << this->m_R[1] << std::endl;
                std::cout << "P[1]: " << this->m_P[1] << std::endl;
                std::cout << "Q: " << this->m_Q << std::endl;

                this->m_valid = true;
            }

            void reset() override {
                this->m_R[0].clear();
                this->m_R[1].clear();
                this->m_P[0].clear();
                this->m_P[1].clear();
                this->m_Q.clear();

                /*
                this->m_R[0].template at<float>(0) = 0.967757881f;
                this->m_R[0].template at<float>(1) = 0.251882195f;
                this->m_R[0].template at<float>(3) = -0.251882195f;
                this->m_R[0].template at<float>(4) = 0.967757881f;
                this->m_R[0].template at<float>(8) = 1.0f;

                this->m_R[1].template at<float>(0) = 0.967757881f;
                this->m_R[1].template at<float>(1) = 0.251882195f;
                this->m_R[1].template at<float>(3) = -0.251882195f;
                this->m_R[1].template at<float>(4) = 0.967757881f;
                this->m_R[1].template at<float>(8) = 1.0f;

                this->m_P[0].template at<float>(0) = 3806.82397f;
                this->m_P[0].template at<float>(2) = 222.984985f;
                this->m_P[0].template at<float>(5) = 3806.82397f;
                this->m_P[0].template at<float>(6) = 661.948059f;
                this->m_P[0].template at<float>(10) = 1.0f;

                this->m_P[1].template at<float>(0) = 3806.82397f;
                this->m_P[1].template at<float>(2) = 222.984985f;
                this->m_P[1].template at<float>(3) = 287156.688f;
                this->m_P[1].template at<float>(5) = 3806.82397f;
                this->m_P[1].template at<float>(6) = 661.948059f;
                this->m_P[1].template at<float>(10) = 1.0f;
                 */

                this->m_valid = false;
            }
        };
    }
}

#endif //EYETRIBE_STEREORECTIFICATION_H
