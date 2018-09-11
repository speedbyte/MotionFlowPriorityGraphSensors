//
// Created by veikas on 01.02.18.
//

#ifndef MAIN_FLOWIMAGEEXTENDED_H
#define MAIN_FLOWIMAGEEXTENDED_H

#include <math.h>
#include <kitti/io_flow.h>


using namespace std;

class FlowImageExtended : public FlowImage {

public:

    FlowImageExtended() {}

    // construct flow image from png file
    FlowImageExtended(const std::string filename) : FlowImage(filename) {}

    FlowImageExtended(const FlowImage &F) : FlowImage(F) {}

    FlowImageExtended(const int32_t width, const int32_t height) : FlowImage(width, height) {

        for (int32_t i=0; i<width*height; i++)
            data_[i*3+2] = -512.0f;
    }

    FlowImageExtended(const float *data, const int32_t width, const int32_t height) : FlowImage(data, width, height) {}

    // set optical flow at given pixel to valid / invalid
    inline void setObjectId(const int32_t u, const int32_t v, const float objId) {
        data_[3*(v*width_+u)+2] = objId;
    }

    // get optical flow u-component at given pixel
    inline float getObjectId (const int32_t u,const int32_t v) {
        return data_[3*(v*width_+u)+2];
    }



    void writeFlowField (const std::string file_name) override {

        png::image< png::rgb_pixel_16 > image(width_,height_);
        for (int32_t v=0; v<height_; v++) {
            for (int32_t u=0; u<width_; u++) {
                png::rgb_pixel_16 val;
                val.red   = (uint16_t)std::max(std::min(-512*64.0f+32768.0f,65535.0f),0.0f);
                val.green = (uint16_t)std::max(std::min(-512*64.0f+32768.0f,65535.0f),0.0f);
                val.blue  = (uint16_t)std::max(std::min(-512*64.0f+32768.0f,65535.0f),0.0f);
                if (getObjectId(u,v) != -512) {
                    // -512 to 512 displacement values are scaled within 0 65535
                    // 32768 in flow file is 0 displacement
                    // 0 in flow file is -512 displacement.
                    // 65535 in flow file is 512 displacement
                    val.red   = (uint16_t)std::max(std::min(getFlowU(u,v)*64.0f+32768.0f,65535.0f),0.0f);
                    val.green = (uint16_t)std::max(std::min(getFlowV(u,v)*64.0f+32768.0f,65535.0f),0.0f);
                    val.blue  = (uint16_t)std::max(std::min(getObjectId(u,v)*64.0f+32768.0f,65535.0f),0.0f);
                }
                image.set_pixel(u,v,val);
            }
        }
        image.write(file_name);
    }

    void writeFalseColors (const std::string file_name, const float max_flow) override {
        float n = 8; // multiplier
        png::image< png::rgb_pixel > image(width_,height_);
        for (int32_t v=0; v<height_; v++) {
            for (int32_t u=0; u<width_; u++) {
                float r=0,g=0,b=0;
                if (getObjectId(u,v) != -512) {
                    float mag = getFlowMagnitude(u,v);
                    float dir = atan2(getFlowV(u,v),getFlowU(u,v));
                    float h   = fmod(dir/(2.0*M_PI)+1.0,1.0);
                    float s   = std::min(std::max(mag*n/max_flow,0.0f),1.0f);
                    float v   = std::min(std::max(n-s,0.0f),1.0f);
                    hsvToRgb(h,s,v,r,g,b);
                }
                image.set_pixel(u,v,png::rgb_pixel(r*255.0f,g*255.0f,b*255.0f));
            }
        }
        image.write(file_name);
    }

    void readFlowField (const std::string file_name) {
        png::image< png::rgb_pixel_16 > image(file_name);
        width_  = image.get_width();
        height_ = image.get_height();
        data_   = (float*)malloc(width_*height_*3*sizeof(float));
        for (int32_t v=0; v<height_; v++) {
            for (int32_t u=0; u<width_; u++) {
                png::rgb_pixel_16 val = image.get_pixel(u,v);
                if (val.blue!=0) {
                    setFlowU(u,v,((float)val.red-32768.0f)/64.0f);
                    setFlowV(u,v,((float)val.green-32768.0f)/64.0f);
                    setObjectId(u,v,((float)val.blue-32768.0f)/64.0f);
                } else {
                    setFlowU(u,v,0);
                    setFlowV(u,v,0);
                    setValid(u,v,0);
                }
            }
        }
    }

    // set optical flow u-component at given pixel
    inline void setFlowU (const int32_t u,const int32_t v,const float val) override {
        data_[3*(v*width_+u)+0] = val;
    }

    // set optical flow v-component at given pixel
    inline void setFlowV (const int32_t u,const int32_t v,const float val) override {
        data_[3*(v*width_+u)+1] = val;
    }

    // set optical flow at given pixel to valid / invalid
    inline void setValid (const int32_t u,const int32_t v,const bool valid) override {
        data_[3*(v*width_+u)+2] = valid; //? 1 : 0;
    }

    void interpolateBackground() override;
};

#endif //MAIN_FLOWIMAGEEXTENDED_H
