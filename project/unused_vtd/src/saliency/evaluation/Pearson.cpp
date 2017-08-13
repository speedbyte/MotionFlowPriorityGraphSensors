//
// Created by geislerd on 10.03.17.
//

#include <unused/saliency/evaluation/Pearson.h>

namespace vtd_framework {
    namespace saliency {
        namespace evaluation {
            void _PearsonImpl::pearson(cv::Mat1f saliency, cv::Mat1f fixation,float& correl) {
                cv::Scalar meanSaliency;
                cv::Scalar meanFixation;
                cv::Scalar stdDevSaliency;
                cv::Scalar stdDevFixation;
                cv::Mat1f normSaliency;
                cv::Mat1f normFixation;
                cv::Mat1f correlOut;

                cv::meanStdDev(saliency,meanSaliency,stdDevSaliency);
                cv::meanStdDev(fixation,meanFixation,stdDevFixation);

                cv::subtract(saliency,meanSaliency,normSaliency);
                cv::subtract(fixation,meanFixation,normFixation);

                cv::divide(normSaliency,stdDevSaliency,normSaliency);
                cv::divide(normFixation,stdDevFixation,normFixation);

                cv::matchTemplate(normSaliency,normFixation,correlOut,cv::TM_CCOEFF_NORMED);

                correl = correlOut.at<float>(0);
            }
        }
    }
}