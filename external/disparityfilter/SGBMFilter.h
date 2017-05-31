/*
 * SGBMFilter.h
 *
 *  Created on: 06.08.2013
 */

#ifndef SGBMFILTER_H_
#define SGBMFILTER_H_

#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp> // get

#include "ImageUtils.h"
#include "ImageTypes.h"

#include "BaseInputFilter.h"

#include "IPluginDescription.h"

#include "GlobalObjectRegistry.h"

namespace io = boost::iostreams;





class SGBMFilter : public BaseInputFilter
{
public:
    SGBMFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

    virtual void process();

protected:
    virtual void fillDefaultParameters();
	virtual void fetchParamValues();
    virtual void init();

private:
    void intParameter(int& var, int pos, int defaultVal);
    int numberOfDisparities, SADWindowSize, preFilterCap, P1, P2, minDisparity, uniquenessRatio, speckleWindowSize, speckleRange, disp12MaxDiff;
    bool fullDP;

    void applySGBMFilter();
};

PLUGIN_DESCRIPTION(SGBMFilter, "{5aa4a836-cef3-45bd-b68e-f34204a05086}", Prepairing, Filter, "generates a disparity map out of a stereoimage", "SGBM Disparity Filter")


#endif /* SGBMFILTER_H_ */
