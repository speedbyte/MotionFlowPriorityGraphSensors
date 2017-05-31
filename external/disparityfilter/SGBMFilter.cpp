#include "SGBMFilter.h"
#include "ParameterList.h"
#include "PluginIOInput.h"
#include "PluginIOOutput.h"
#include <opencv2/opencv.hpp>

using namespace cv;

using namespace ImageStreaming;

SGBMFilter::SGBMFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters) : BaseInputFilter(gobjreg, parameters)
{
    std::shared_ptr<PluginIOInput<ImageData>> input1(new PluginIOInput<ImageData>("Left Image", this));
    input1->setStreamType(IMAGE);
    io_collection->addInput(input1);

	std::shared_ptr<PluginIOInput<ImageData>> input2(new PluginIOInput<ImageData>("Right Image", this));
	input2->setStreamType(IMAGE);
    io_collection->addInput(input2);

    std::shared_ptr<PluginIOOutput<ImageData>> output1(new PluginIOOutput<ImageData>("Left Image", this));
    output1->setStreamType(IMAGE);
    io_collection->addOutput(output1);

	std::shared_ptr<PluginIOOutput<ImageData>> output2(new PluginIOOutput<ImageData>("Right Image", this));
	output2->setStreamType(IMAGE);
    io_collection->addOutput(output2);

	std::shared_ptr<PluginIOOutput<ImageData>> output3(new PluginIOOutput<ImageData>("Disparity", this));
	output3->setStreamType(IMAGE);
    io_collection->addOutput(output3);
}

void SGBMFilter::init()
{
	fetchParamValues();
}

void SGBMFilter::fillDefaultParameters()
{
    m_parameters.add(192, "numbers of disparities");
    m_parameters.add(9, "SAD Window size", 3, 11);
    m_parameters.add(0, "pre filter cap", 0, 400);
    m_parameters.add(600, "P1");
    m_parameters.add(2400, "P2");
    m_parameters.add(0, "min Disparity");
    m_parameters.add(1, "uniquenessRatio", 0, 100 );
    m_parameters.add(0, "speckle window size");
    m_parameters.add(0, "speckle range");
    m_parameters.add(10, "disp12MaxDiff");
	fullDP = false;
}

void SGBMFilter::fetchParamValues(){
	unsigned int i = 0;
	numberOfDisparities = m_parameters.getInt(i++);
	SADWindowSize = m_parameters.getInt(i++);
	preFilterCap = m_parameters.getInt(i++);
	P1 = m_parameters.getInt(i++);
	P2 = m_parameters.getInt(i++);
	minDisparity = m_parameters.getInt(i++);
	uniquenessRatio = m_parameters.getInt(i++);
	speckleWindowSize = m_parameters.getInt(i++);
	speckleRange = m_parameters.getInt(i++);
	disp12MaxDiff = m_parameters.getInt(i++);
}

void SGBMFilter::intParameter(int& var, int pos, int defaultVal)
{
    if(m_parameters.getSize() > 0 && m_parameters.getInt(pos) >= 0) {
        var = m_parameters.getInt(pos);
    } else {
        var = defaultVal;
    }
}

void SGBMFilter::process(){
	applySGBMFilter();
}

void SGBMFilter::applySGBMFilter(){
    // Get data
	ImageData left_in = io_collection->getInputData<ImageData>("Left Image");
	ImageData right_in = io_collection->getInputData<ImageData>("Right Image");
	ImageData depth_out;

    //Calculate depth
    Mat left;
    Mat right;

	if (left_in.pf == ImageStreaming::Mono8){
		left = Mat(left_in.height, left_in.width, CV_8UC1);
		ImageStreaming::ImageUtils::Mono8ImageToUC1Mat(left, left_in.ptr);
	}
	else if (left_in.pf == ImageStreaming::RGB888){
		left = Mat(left_in.height, left_in.width, CV_8UC3);
		ImageStreaming::ImageUtils::RGB888ImageToUC3Mat(left, left_in.ptr);
	}

	if (right_in.pf == ImageStreaming::Mono8){
		right = Mat(right_in.height, right_in.width, CV_8UC1);
		ImageStreaming::ImageUtils::Mono8ImageToUC1Mat(right, right_in.ptr);
	}
	else if (right_in.pf == ImageStreaming::RGB888){
		right = Mat(right_in.height, right_in.width, CV_8UC3);
		ImageStreaming::ImageUtils::RGB888ImageToUC3Mat(right, right_in.ptr);
	}

    Mat disp, disp8;

    Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create(
                minDisparity,
                numberOfDisparities,
                SADWindowSize,
                P1,
                P2,
                disp12MaxDiff,
                0, // needs own parameter?
                uniquenessRatio,
                speckleWindowSize,
                speckleRange,
                fullDP
                );

    /*cv::StereoSGBM sgbm;
    sgbm.preFilterCap = preFilterCap;
    sgbm.SADWindowSize = SADWindowSize;
    sgbm.P1 = P1;
    sgbm.P2 = P2;
    sgbm.minDisparity = minDisparity;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = uniquenessRatio;
    sgbm.speckleWindowSize = speckleWindowSize;
    sgbm.speckleRange = speckleRange;
    sgbm.disp12MaxDiff = disp12MaxDiff;
    sgbm.fullDP = fullDP;
    */


    sgbm->compute(left, right, disp);
    disp.convertTo(disp8, CV_32F, 1 / 16.f);

	depth_out.pf = Float32;
	depth_out.width = disp8.cols;
	depth_out.height = disp8.rows;
	depth_out.colordepth = sizeof(float);
	depth_out.ptr = (image_t)malloc(disp.rows * disp.cols * sizeof(float));

	ImageStreaming::ImageUtils::F32MatToImage(disp8, depth_out.ptr);

	io_collection->setOutputData<ImageData>("Left Image", left_in);
	io_collection->setOutputData<ImageData>("Right Image", right_in);
	io_collection->setOutputData<ImageData>("Disparity", depth_out);
}
