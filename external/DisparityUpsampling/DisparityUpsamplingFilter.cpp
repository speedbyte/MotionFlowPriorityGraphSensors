#include "DisparityUpsamplingFilter.h"

#include "PluginIOInput.h"
#include "PluginIOOutput.h"

using namespace ImageStreaming;

DisparityUpsamplingFilter::DisparityUpsamplingFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters) : BaseInputFilter(gobjreg, parameters)
{
    std::shared_ptr<PluginIOInput<ImageData>> input1(new PluginIOInput<ImageData>("ImageData1", this));
    input1->setStreamType(IMAGE);
    io_collection->addInput(input1);

    std::shared_ptr<PluginIOOutput<ImageData>> output1(new PluginIOOutput<ImageData>("ImageData1", this));
    output1->setStreamType(IMAGE);
    io_collection->addOutput(output1);
}

void DisparityUpsamplingFilter::process(){
	applyDisparityUpsamplingFilter();
}

void DisparityUpsamplingFilter::applyDisparityUpsamplingFilter(){
    // Get input
    std::shared_ptr<PluginIOInput<ImageData>> inPtr = std::dynamic_pointer_cast<PluginIOInput<ImageData>>(io_collection->getInput("ImageData1"));

    if(inPtr->getPluginIOType() == PULL) read();

    // Get data
    ImageData img;
    img = inPtr->getData();



    //DO DISPARITY UPSAMPLING HERE! Take a look at DisparityRefinementFilter - it is similar.



    // Get output
    std::shared_ptr<PluginIOOutput<ImageData>> outPtr = std::dynamic_pointer_cast<PluginIOOutput<ImageData>>(io_collection->getOutput("ImageData1"));

    outPtr->setData(img);
}

void DisparityUpsamplingFilter::fillDefaultParameters()
{
	//Need some parameters in the GUI? Add here!
    m_parameters.add(1.0f, "Test parameter", 0.0f, 10.0f);
}

void DisparityUpsamplingFilter::fetchParamValues(){
	unsigned int i = 0;
	test_param = m_parameters.getFloat(i++);
}
