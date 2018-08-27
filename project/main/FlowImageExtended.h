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

    }

    FlowImageExtended(const float *data, const int32_t width, const int32_t height) : FlowImage(data, width, height) {}

    // set optical flow at given pixel to valid / invalid
    inline void setObjectId(const int32_t u, const int32_t v, const float objId) {
        data_[3*(v*width_+u)+2] = objId;
    }

    // write flow field to png file
    void writeExtended (const std::string file_name) {
        writeFlowField (file_name);
    }

    // get optical flow u-component at given pixel
    inline float getObjectId (const int32_t u,const int32_t v) {
        return data_[3*(v*width_+u)+2];
    }

    void writeFlowField (const std::string file_name) {

        png::image< png::rgb_pixel_16 > image(width_,height_);
        for (int32_t v=0; v<height_; v++) {
            for (int32_t u=0; u<width_; u++) {
                png::rgb_pixel_16 val;
                val.red   = 0;
                val.green = 0;
                val.blue  = 0;
                if (isValid(u,v)) {
                    val.red   = (uint16_t)std::max(std::min(getFlowU(u,v)*64.0f+32768.0f,65535.0f),0.0f);
                    val.green = (uint16_t)std::max(std::min(getFlowV(u,v)*64.0f+32768.0f,65535.0f),0.0f);
                    val.blue  = (uint16_t)getObjectId(u,v);
                }
                image.set_pixel(u,v,val);
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
                if (val.blue>0) {
                    setFlowU(u,v,((float)val.red-32768.0f)/64.0f);
                    setFlowV(u,v,((float)val.green-32768.0f)/64.0f);
                    setValid(u,v,val.blue);
                } else {
                    setFlowU(u,v,0);
                    setFlowV(u,v,0);
                    setValid(u,v,0);
                }
            }
        }
    }

    // set optical flow u-component at given pixel
    inline void setFlowU (const int32_t u,const int32_t v,const float val) {
        data_[3*(v*width_+u)+0] = val;
    }

    // set optical flow v-component at given pixel
    inline void setFlowV (const int32_t u,const int32_t v,const float val) {
        data_[3*(v*width_+u)+1] = val;
    }

    // set optical flow at given pixel to valid / invalid
    inline void setValid (const int32_t u,const int32_t v,const bool valid) {
        data_[3*(v*width_+u)+2] = valid; //? 1 : 0;
    }

};

#endif //MAIN_FLOWIMAGEEXTENDED_H
