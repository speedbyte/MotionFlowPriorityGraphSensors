//
// Created by geislerd on 21.02.17.
//

#include <unused/saliency/activation/Spectral.h>

namespace vtd_framework {
    namespace saliency {
        namespace activation {

            cv::Mat1f _SpectralImpl::wrap2pi(cv::Mat1f in) {
                cv::Mat1f out;
                float f;
                out = cv::Mat1f(in.size());
                for (int x = 0; x < in.cols; x++)
                    for (int y = 0; y < in.rows; y++) {
                        f = in(y, x);
                        if (f > 0)
                            out(y, x) = fmodf(f + float(M_PI), 2.0f * float(M_PI)) - float(M_PI);
                        else
                            out(y, x) = fmodf(f - float(M_PI), 2.0f * float(M_PI)) + float(M_PI);
                    }
                return out;
            }
            cv::Mat1f _SpectralImpl::sin(cv::Mat1f in) {
                cv::Mat1f out;

                out = this->wrap2pi(in);
                cv::subtract(out, out.mul(out.mul(out)) / 6, out); // x' = x - (x^3)/6

                return out;
            }
            cv::Mat1f _SpectralImpl::cos(cv::Mat1f in) {
                cv::Mat1f out;

                cv::subtract(M_PI_2,in,out);
                return this->sin(out);
            }
            void _SpectralImpl::spectralWhitening(cv::Mat1f in, cv::Mat1f out, cv::Size fz, float whitening, float smooth) {
                cv::Mat2f fft;
                cv::Mat1f xy[2];
                cv::Mat1f pha, mag, mag_blur, sr;


                // calculate fourier transformation
                cv::dft(in, fft, cv::DFT_COMPLEX_OUTPUT);
                cv::split(fft, xy);

                // calculate phase
                cv::phase(xy[0], xy[1], pha);

                if(whitening > 0.0f) {
                    // calculate magnitude
                    cv::magnitude(xy[0], xy[1], mag);

                    // calculate SR
                    cv::log(mag, mag);
                    cv::GaussianBlur(mag, mag_blur, fz, whitening);
                    cv::subtract(mag, mag_blur, sr);
                    cv::exp(sr, sr);
                } else
                    sr = cv::Mat1f::ones(in.rows,in.cols);

                // reassemble fft using fast sin approximation:
                // x' = x - (x^3)/6
                xy[0] = sr.mul(this->cos(pha));
                xy[1] = sr.mul(this->sin(pha));
                cv::merge(xy, 2, fft);

                // inverse fft
                cv::idft(fft, fft, cv::DFT_COMPLEX_OUTPUT);
                cv::split(fft, xy);
                cv::magnitude(xy[0], xy[1], mag);
                cv::pow(mag, 2, out);

                // blur
                if(smooth > 0.0f) {
                    out(0) = 0.0f;
                    cv::GaussianBlur(out, out, fz, smooth);
                }
            }
        }
    }
}