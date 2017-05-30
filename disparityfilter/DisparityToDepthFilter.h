/*
 * DISPARITYTODEPTHFilter.h
 *
 *  Created on: 06.08.2013
 */

#ifndef DISPARITYTODEPTHFILTER_H_
#define DISPARITYTODEPTHFILTER_H_

#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp> // get
#include <opencv2/opencv.hpp>


#include "ImageUtils.h"
#include "ImageTypes.h"

#include "../PluginChain/BaseInputFilter.h"

#include "IPluginDescription.h"

#include "GlobalObjectRegistry.h"

namespace io = boost::iostreams;

using namespace cv;


class DisparityToDepthFilter : public BaseInputFilter
{
public:
    DisparityToDepthFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

    virtual void process();

protected:
    virtual void fillDefaultParameters();
	virtual void fetchParamValues();
    virtual void init();

private:
	int focalLength, cameraDistance, minDisparity;
	double pixelSize;

    float calcDepth(float disparity);

    unsigned char mapToChar(double value, double max);

    void applyDisparityToDepthFilter();
};

PLUGIN_DESCRIPTION(DisparityToDepthFilter, "{faf87b59-30ba-49db-9892-0357e62e095f}",Prepairing, Filter, "generates depthmap from a disparity map", "Disparity2Depthmap")

#endif /* DISPARITYTODEPTHFILTER_H_ */
