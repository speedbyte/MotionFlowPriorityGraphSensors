/*
* VTDSource.h - A source for shared memory access to Vires VTD
*/

#ifndef VTDSourceH
#define VTDSourceH

#include <vector>
#include <iosfwd>                          // streamsize
#include <sstream>
#include <iomanip>
#include <boost/iostreams/categories.hpp>  // source_tag

#include "../Parameters/ParameterList.h"
#include "../Parameters/ParameterImpl.h"
#include "../PluginChain/BaseSource.h"

#include "IPluginDescription.h"

#include <QMultiMap>

#include "VTDConnector.h"

//#include "TrafficSigns.h"
//#include "Lanes.h"

namespace io = boost::iostreams;

namespace ImageStreaming {


	//! source for single images
	class VTDSource : public BaseSource{
	public:

		//! create instance of single image source
		//! @param parameters	[0] directory containing png images [1] suffix of the image
		VTDSource(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

		//! read data from source
		//! @param s	stream to write on
		//! @param n 	count of requested bytes
		//! @return 	returns the count of written bytes

		virtual bool open();
        virtual void close();
        virtual bool load();

  protected:
		virtual void fillDefaultParameters();
		virtual void init();

		virtual void fetchParamValues();

private:

    void handleImg(size_t shmNum, RDB_MSG_t* img);
    void handleSimInfo(RDB_MSG_t* package);

    //Handler for specific types
    void handleTrafficSigns(RDB_MSG_t* package);
    void handleRoadMarks(RDB_MSG_t* package);

    void printImageInfo(RDB_IMAGE_t* img);
    void convertToDepth(ImageData* image, RDB_CAMERA_t* cameraInfo);
    void copyBuffToImageData(RDB_IMAGE_t* buf_img, ImageData& image, RDB_MSG_t* pRdbMsg);

    //Connection handling
    VTDConnector* m_vtdConnector;

    //Flags for VTD
    const int FREE_FLAG = RDB_SHM_BUFFER_FLAG_TC;
    const unsigned int SHM_IMG_BUFFER = 0x0811b;

    //Signals data ready to be streamed if true, otherwise data ready to be fetched from VTD
    bool data_ready;

    //TrafficSign GT data
    QMultiMap<unsigned int, TrafficSign*> ts_gt;
    QMultiMap<unsigned int, Lane*> lane_gt;

    //Store previous frame to signal frame dropping
    unsigned int previous_frame_nr;
  };


  PLUGIN_DESCRIPTION(VTDSource, "{a94f2a49-5ead-46a7-ba3e-6747e3efec6e}", Prepairing, Source, "VTD - shared memory access source", "VTD Source")

} /* namespace ImageStreaming */



#endif
