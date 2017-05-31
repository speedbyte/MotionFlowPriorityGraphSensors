
#include "DisparityRefinementFilter.h"
#include "PluginIOInput.h"
#include "PluginIOOutput.h"


using namespace ImageStreaming;

DisparityRefinementFilter::DisparityRefinementFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters) : BaseInputFilter(gobjreg, parameters)
{
    std::shared_ptr<PluginIOInput<ImageData>> input1(new PluginIOInput<ImageData>("Disparity In", this));
    input1->setStreamType(IMAGE);
    io_collection->addInput(input1);

    std::shared_ptr<PluginIOOutput<ImageData>> output1(new PluginIOOutput<ImageData>("Disparity Out", this));
    output1->setStreamType(IMAGE);
    io_collection->addOutput(output1);
}

void DisparityRefinementFilter::medianFilter(int kernel_size, float* disp, unsigned int width, unsigned int height){
    for (size_t i = 0; i < height; i++) {
        for (size_t j = 0; j < width; j++) {
            QList<float> neighborhood;
            for (int h = -kernel_size/2; h <= kernel_size/2; h++)
                for (int k = -kernel_size/2; k <= kernel_size/2; k++) {
                    int y = i + h;
                    if (y < 0) y = 0;
                    if (static_cast<unsigned int>(y) >= height) y = height - 1; //To remove compiler warning
                    int x = j + k;
					if (x < 0) x = 0; 
					if (static_cast<unsigned int>(x) >= width) x = width - 1; //To remove compiler warning
                    neighborhood << disp[y * width + x];
                }
            disp[i * width + j] = median(neighborhood);
        }
    }
}

unsigned int DisparityRefinementFilter::median(QList<float>& list){
	int N = list.size();
    qSort(list);
    if (N % 2 == 0)
        return (list[N / 2 - 1] + list[N / 2]) / 2;
    return list[N / 2];
}

void DisparityRefinementFilter::fillDefaultParameters()
{
	UIntParameter p(10,"Disparity threshold",0,255);
	m_parameters.add(p);
}

void DisparityRefinementFilter::fetchParamValues(){
	unsigned int i = 0;
	threshold = m_parameters.getUInt(i++);
}

void DisparityRefinementFilter::process(){
	applyDisparityRefinementFilter();
}

void DisparityRefinementFilter::applyDisparityRefinementFilter(){
   
    // Get data
    ImageData img = io_collection->getInputData<ImageData>("Disparity In");

    unsigned int width = img.width;
    unsigned int height = img.height;

    image_t d = img.ptr;
    float* disp = reinterpret_cast<float*>(d);

    //Medianfilter
    medianFilter(9, disp, width, height);

    //Lookup map - how many neighboring pixels are valid?
    unsigned char* lookup_map = new unsigned char[width * height];

    //Threshold depth and init lookup map
    /*for (unsigned int x = 0; x < width; ++x){
        for (unsigned int y = 0; y < height; ++y){
            if (disp[y * width + x] <= threshold && disp[y * width + x] >= 0) disp[y * width + x] = -1.f;
        }
    }*/
    for (unsigned int x = 0; x < width; ++x){
        for (unsigned int y = 0; y < height; ++y){
            //Pixel is valid
            if (disp[y * width + x] >= 0) lookup_map[y * width + x] = 9;
            //Pixel is invalid - count valid neighbors
            else{
                unsigned char sum = 0;
                for (int i = -1; i <= 1; ++i){
                    for (int j = -1; j <= 1; ++j){
                        if (y + j >= height || x + i >= width) continue;
                        if (disp[(y + j) * width + (x + i)] >= 0) sum++;
                    }
                }
                lookup_map[y * width + x] = sum;
            }
        }
    }

    int current_pixel_group = 8;

    while (current_pixel_group > -1){
        bool pixel_found = false;
        for (unsigned int x2 = 0; x2 < width; ++x2){
            for (unsigned int y2 = 0; y2 < height; ++y2){
                if (lookup_map[y2 * width + x2] == current_pixel_group){
                    pixel_found = true;

                    //Interpolate
                    float val = 0;
                    unsigned int val_count = 0;
                    for (int i = -1; i <= 1; ++i){
                        for (int j = -1; j <= 1; ++j){
                            if (y2 + j >= height || x2 + i >= width) continue;
                            if (float v = disp[(y2 + j) * width + (x2 + i)] >= 0){
                                val += v;
                                val_count++;
                            }
                            if (lookup_map[(y2 + j) * width + (x2 + i)] != 9) lookup_map[(y2 + j) * width + (x2 + i)] += 1;
                        }
                    }

                    disp[y2 * width + x2] = (1.0f / val_count) * val;

                    //Update lookup map
                    lookup_map[y2 * width + x2] = 9;
                }
            }
        }
        if (pixel_found){
            //Reset pixel group to 7, reset x and y
            current_pixel_group = 8;
        }
        else current_pixel_group--;
    }

    delete[] lookup_map;

    io_collection->setOutputData<ImageData>("Disparity Out", img);
}
