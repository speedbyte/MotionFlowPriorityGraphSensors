//
// Created by geislerd on 13.03.17.
//

#include <unused/saliency/activation/BooleanMaps.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            void _BooleanMapsImpl::floodFillBorder(cv::Mat1f &map) {
                const cv::Point seedPoint(0, 0);

                /*
                 *  draw border
                 */
                for (cv::Point i(0, 0); i.x < map.cols; i.x++)
                    map(i) = 1.0f;
                for (cv::Point i(0, map.rows - 1); i.x < map.cols; i.x++)
                    map(i) = 1.0f;
                for (cv::Point i(0, 0); i.y < map.rows; i.y++)
                    map(i) = 1.0f;
                for (cv::Point i(map.cols - 1, 0); i.y < map.rows; i.y++)
                    map(i) = 1.0f;

                /*
                 *  delete segments with border contact
                 */
                cv::floodFill(map, seedPoint, 0);
            }

            void _BooleanMapsImpl::add(cv::Mat1f map, cv::Mat1f out) {
                // remove segments at image border
                this->floodFillBorder(map);

                cv::add(out, map, out);
            }

            void _BooleanMapsImpl::threshold(cv::Mat1f mat, cv::Mat1f out, double hmin, double hmax, int n, int type) {
                double thresh_step;
                cv::Mat1f m;

                thresh_step = (hmax - hmin) / n;

                for (double thresh = hmin + thresh_step; thresh < hmax; thresh += thresh_step) {
                    cv::threshold(mat, m, thresh, 1.0f, type);
                    this->add(m, out);
                }
            }

            void _BooleanMapsImpl::booleanMaps(cv::Mat1f in, cv::Mat1f out, cv::Size sz, float sigma, int n) {
                double hmin, hmax, r;
                cv::Mat1f in_tmp;

                // smooth input to reduce noise
                cv::GaussianBlur(in, in_tmp, sz, sigma, 0, cv::BORDER_REPLICATE);

                // calculate threshold
                cv::minMaxLoc(in_tmp, &hmin, &hmax);
                r = (hmax - hmin) * 0.25f;
                this->threshold(in_tmp, out, hmin + r, hmin + r * 2.0f, n, cv::THRESH_BINARY_INV);
                this->threshold(in_tmp, out, hmax - r * 2.0f, hmax - r, n, cv::THRESH_BINARY);
            }
        }
    }
}