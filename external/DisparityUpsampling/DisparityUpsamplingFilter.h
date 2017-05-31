#ifndef BRIGHTNESSFILTERPLUGIN_H
#define BRIGHTNESSFILTERPLUGIN_H

#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp> // get
#include "boost/iostreams/read.hpp"
#include <boost/ref.hpp>

#include "BaseInputFilter.h"
#include "Exceptions.h"
#include "IPluginDescription.h"
#include "IPluginContainer.h"

#include "GlobalObjectRegistry.h"

class DisparityUpsamplingFilter : public BaseInputFilter
{
public:
    //! create an instance of DisparityUpsampling filter
	DisparityUpsamplingFilter(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

	virtual ~DisparityUpsamplingFilter(){}

    virtual void fillDefaultParameters();

    virtual void process();

protected:
	virtual void fetchParamValues();

private:
    static ParameterList s_standardParameters;

	float test_param;

    void applyDisparityUpsamplingFilter();
};

PLUGIN_DESCRIPTION(DisparityUpsamplingFilter, "{af484c35-12c0-453b-972b-880702155215}", Prepairing, Filter, "Creates dense disparity maps", "Disparity Upsampling Filter")


#endif
