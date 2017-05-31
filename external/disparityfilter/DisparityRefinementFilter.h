/*
 * DisparityRefinementFilter.h
 *
 *  Created on: 06.08.2013
 */

#ifndef DISPARITYREFINEMENTFILTER_H_
#define DISPARITYREFINEMENTFILTER_H_

#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp> // get
#include <opencv2/opencv.hpp>
#include <vector>

#include "ImageUtils.h"
#include "ImageTypes.h"

#include "../PluginChain/BaseInputFilter.h"

#include "IPluginDescription.h"

#include "GlobalObjectRegistry.h"

namespace io = boost::iostreams;

using namespace cv;


class DisparityRefinementFilter : public BaseInputFilter
{
public:
    DisparityRefinementFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

    virtual void process();

protected:
    virtual void fillDefaultParameters();
	virtual void fetchParamValues();

private:
	unsigned int threshold;

    void medianFilter(int kernel_size, float* disp, unsigned int width, unsigned int height);
	unsigned int median(QList<float>& list);

    void applyDisparityRefinementFilter();
};

PLUGIN_DESCRIPTION(DisparityRefinementFilter, "{807fd711-d801-4ad2-bd27-6a2e35861627}", Prepairing, Filter, "fills holes in disparity map", "Disparity refinement filter")

#endif /* DISPARITYREFINEMENTFILTER_H_ */
