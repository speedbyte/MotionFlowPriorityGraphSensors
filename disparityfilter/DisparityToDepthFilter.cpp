#include "DisparityToDepthFilter.h"
#include "PluginIOInput.h"
#include "PluginIOOutput.h"

using namespace ImageStreaming;


DisparityToDepthFilter::DisparityToDepthFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters): BaseInputFilter(gobjreg, parameters)
{
    std::shared_ptr<PluginIOInput<ImageData>> input1(new PluginIOInput<ImageData>("Disparity", this));
    input1->setStreamType(IMAGE);
    io_collection->addInput(input1);

    std::shared_ptr<PluginIOOutput<ImageData>> output1(new PluginIOOutput<ImageData>("Depth", this));
    output1->setStreamType(IMAGE);
    io_collection->addOutput(output1);
}

void DisparityToDepthFilter::init()
{

}

// Calculate depth from disparity in [m]
float DisparityToDepthFilter::calcDepth(float disparity)
{
    if(disparity <= minDisparity) {
		return (focalLength*cameraDistance) / (minDisparity*(pixelSize));
    } else {
        return (focalLength*cameraDistance)/(disparity*(pixelSize));
    }
}

unsigned char DisparityToDepthFilter::mapToChar(double value, double max)
{
    if(value > max) {
        value = max;
    }
    return (unsigned char)(value/1000.0f);
}

void DisparityToDepthFilter::fillDefaultParameters()
{
    m_parameters.add(UIntParameter(6, "focal length [mm]"));
    m_parameters.add(UIntParameter(120, "distance [mm]"));
    m_parameters.add(FloatParameter(3.75, "pixel size [µm]"));
    m_parameters.add(UIntParameter(1, "min disparity"));
}

void DisparityToDepthFilter::fetchParamValues(){
	unsigned int i = 0;
	focalLength = m_parameters.getUInt(i++);
	cameraDistance = m_parameters.getUInt(i++);
	pixelSize = m_parameters.getFloat(i++);
	minDisparity = m_parameters.getUInt(i++);
}

void DisparityToDepthFilter::process(){
	applyDisparityToDepthFilter();
}

void DisparityToDepthFilter::applyDisparityToDepthFilter(){
    
	ImageData img = io_collection->getInputData<ImageData>("Disparity");

    Mat frame = Mat(img.height, img.width, CV_32F);
    ImageStreaming::ImageUtils::ImageToF32Mat(frame, img.ptr);

    //max depth value
    //float max_depth = calcDepth(minDisparity);

    for (int y = 0; y < frame.rows; y++) {
        for (int x = 0; x < frame.cols; x++) {
            float disparity = frame.at<float>(cv::Point(x, y));
            frame.at<float>(cv::Point(x, y)) = calcDepth(disparity);
        }
    }

    ImageStreaming::ImageUtils::F32MatToImage(frame, img.ptr);
	io_collection->setOutputData<ImageData>("Depth", img);
}
