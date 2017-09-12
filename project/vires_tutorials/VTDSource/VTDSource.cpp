#include "VTDSource.h"
#include <QDebug>
#include "ImageUtils.h"
#include <boost/bind.hpp>
#include "RDBHandler.hh"

#ifdef __unix__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

namespace ImageStreaming {
  VTDSource::VTDSource(GlobalObjectRegistry* gobjreg, const ParameterList* parameters) : BaseSource(gobjreg, parameters)
  {
    data_ready = false;

    previous_frame_nr = 0;

    std::shared_ptr<PluginIOOutput<ImageData>> leftImage(new PluginIOOutput<ImageData>("Left Image", this));
    leftImage->setStreamType(IMAGE);
    io_collection->addOutput(leftImage);

    std::shared_ptr<PluginIOOutput<ImageData>> rightImage(new PluginIOOutput<ImageData>("Right Image", this));
    rightImage->setStreamType(IMAGE);
    io_collection->addOutput(rightImage);

    std::shared_ptr<PluginIOOutput<ImageData>> depthImage(new PluginIOOutput<ImageData>("Depth Image", this));
    depthImage->setStreamType(IMAGE);
    io_collection->addOutput(depthImage);

    std::shared_ptr<PluginIOOutput<TrafficSigns>> tsr_gt(new PluginIOOutput<TrafficSigns>("TSR-GT", this));
    tsr_gt->setStreamType(FEATURES);
    io_collection->addOutput(tsr_gt);

    std::shared_ptr<PluginIOOutput<Lanes>> lane_gt(new PluginIOOutput<Lanes>("Lane-GT", this));
    lane_gt->setStreamType(FEATURES);
    io_collection->addOutput(lane_gt);
  }

  void VTDSource::init()
  {
    fetchParamValues();
  }

  bool VTDSource::open(){
#ifdef __unix__
    unsigned int shms[] = {SHM_IMG_BUFFER};
    std::string server = m_parameters.getString(0);
    int port = m_parameters.getInt(1);
    int sensor_port = m_parameters.getInt(2);

    m_vtdConnector = new VTDConnector(1, shms, server, port);
    m_vtdConnector->registerDataCallback(boost::bind(&VTDSource::handleImg, this, _1 ,_2));
    m_vtdConnector->registerInfoCallback(boost::bind(&VTDSource::handleSimInfo, this, _1), 1, &sensor_port);
    std::cout << "VTD opened" << std::endl;
    return m_vtdConnector->open();
#else
    return false;
#endif
  }

  bool VTDSource::load()
  {
    // get image from stream and call handler directly
    while(!data_ready){
        m_vtdConnector->getFrame();
    }

    //Data is now ready to be fetched from VTD again
    data_ready = false;

    //Load successful
    return true;
  }

  void VTDSource::handleImg(size_t /* shmNum */, RDB_MSG_t* pRdbMsg){

        //Current frame number
        unsigned int frame_nr = pRdbMsg->hdr.frameNo;

        /*qDebug() << QThread::currentThreadId() << ": Frame " << frame_nr;
        if(previous_frame_nr && previous_frame_nr != frame_nr - 1){
            qDebug() << " -- WARNING" << frame_nr << "! Frame dropped: " << previous_frame_nr + 1;
        } else qDebug() << " -- OK ";*/#

        if(previous_frame_nr && previous_frame_nr != frame_nr - 1)
            std::cout << " -- WARNING: Frames " << previous_frame_nr + 1 << " to " << frame_nr - 1 << " dropped!" << std::endl;

        //Prepare ImageStream
        if (pRdbMsg->entryHdr.pkgId == RDB_PKG_ID_IMAGE)
        {
            RDB_MSG_ENTRY_HDR_t* p_entry_hdr = &pRdbMsg->entryHdr;
            RDB_IMAGE_t* buf_img = ((RDB_IMAGE_t*) (p_entry_hdr + 1)) + 1;

            //std::cout << "SHM No :" << shmNum << std::endl;
            //printImageInfo(&(pRdbMsg->u.image));

            //Left image
            ImageData leftImage;
            copyBuffToImageData(buf_img, leftImage, pRdbMsg);
            io_collection->setOutputData<ImageData>("Left Image", leftImage);

            //Right image
            ImageData rightImage;
            copyBuffToImageData(buf_img, rightImage, pRdbMsg);
            io_collection->setOutputData<ImageData>("Right Image", rightImage);

            //Increase pointer to new entry header
            p_entry_hdr = (RDB_MSG_ENTRY_HDR_t*) (((char*)p_entry_hdr) + p_entry_hdr->dataSize + p_entry_hdr->headerSize); //CameraInfo
            RDB_CAMERA_t* cameraInfo = ((RDB_CAMERA_t*) (p_entry_hdr + 1));

            p_entry_hdr = (RDB_MSG_ENTRY_HDR_t*) (((char*)p_entry_hdr) + p_entry_hdr->dataSize + p_entry_hdr->headerSize); //Depth header
            buf_img = ((RDB_IMAGE_t*) (p_entry_hdr + 1)) + 1;

            //Depth data
            ImageData depthImage;
            depthImage.width = pRdbMsg->u.image.width;
            depthImage.height = pRdbMsg->u.image.height;
            depthImage.colordepth = sizeof(float);
            depthImage.pf = Float32;
            size_t bufSize = depthImage.width * depthImage.height * sizeof(float);
            unsigned char * depth_buf = (unsigned char *)std::malloc(sizeof(unsigned char) * bufSize);
            memcpy(depth_buf, buf_img, bufSize);
            depthImage.ptr = depth_buf;
            convertToDepth(&depthImage, cameraInfo);
            ImageUtils::hFlipImageData(depthImage);
            io_collection->setOutputData<ImageData>("Depth Image", depthImage);

            //TSR GT
            QList<TrafficSign*> frame_traffic_signs = ts_gt.values(frame_nr);
            TrafficSigns ts_out;

            //Adapt y position - flip
            foreach(TrafficSign* ts, frame_traffic_signs){
                QVector3D pos = ts->getPos();
                pos.setY(leftImage.height - pos.y());
                ts->setPos(pos);
                ts_out.push_back(ts);
            }

            //Remove traffic signs from container
            ts_gt.remove(frame_nr);

            //Set output TSR-GT
            io_collection->setOutputData<TrafficSigns>("TSR-GT", ts_out);

            //Lanes GT
            QList<Lane*> frame_lanes = lane_gt.values(frame_nr);
            Lanes lanes_out;

            //Adapt y position - flip
            foreach(Lane* lane, frame_lanes){
                lane->flipTesselationPoints(leftImage.height);
                lanes_out.push_back(lane);
            }

            //Remove traffic signs from container
            lane_gt.remove(frame_nr);

            //Set output Lane-GT
            io_collection->setOutputData<Lanes>("Lane-GT", lanes_out);

            //Remember last frame
            previous_frame_nr = frame_nr;

            //Set data ready
            data_ready = true;

            //qDebug() << " -- processed." << std::endl;
        }
  }

  void VTDSource::handleSimInfo(RDB_MSG_t* package){
    handleTrafficSigns(package);
    handleRoadMarks(package);
  }

  void VTDSource::handleTrafficSigns(RDB_MSG_t* package){
    TrafficSigns traffic_signs;

    unsigned int frame_nr = package->hdr.frameNo;

    //Find traffic signs and extract info, store in QVector
    unsigned int num_elements = 0;
    RDB_TRAFFIC_SIGN_t* ts_package = (RDB_TRAFFIC_SIGN_t*) Framework::RDBHandler::getFirstEntry(package, RDB_PKG_ID_TRAFFIC_SIGN, num_elements, false);
    while(num_elements && ts_package){
      TrafficSign* ts = new TrafficSign(ts_package->id, package->hdr.frameNo, ts_package->type, ts_package->subType);
      traffic_signs.push_back(ts);
      num_elements--;
      if( num_elements ) ts_package++;
    }

    //Find rest of the info in another package and add to stored object
    num_elements = 0;
    RDB_OBJECT_STATE_BASE_t* os_package = (RDB_OBJECT_STATE_BASE_t*) Framework::RDBHandler::getFirstEntry(package, RDB_PKG_ID_OBJECT_STATE, num_elements, false);
    //std::cout << "Number of elements: " << num_elements << std::endl;
    while(num_elements && os_package){
      //std::cout << "Found ID: " << os_package->id << std::endl;
      foreach(TrafficSign* ts, traffic_signs){
        if(frame_nr == ts->getFrameNumber() && os_package->id == (uint)ts->getID()){
          ts->setPos(QVector3D(os_package->pos.x, os_package->pos.y, os_package->pos.z));
          ts->setSize(QSize(os_package->geo.dimX, os_package->geo.dimY));
        }
      }
      num_elements--;
      if( num_elements ) os_package++;
    }

    foreach(TrafficSign* ts, traffic_signs) ts_gt.insert(frame_nr, ts);

    //io_collection->setOutputData<TrafficSigns>("TSR-GT", traffic_signs);
  }

  void VTDSource::handleRoadMarks(RDB_MSG_t* package){
    unsigned int frame_nr = package->hdr.frameNo;

    //Find roadmarks and extract info, store in QVector
    unsigned int num_elements = 0;
    RDB_ROADMARK_t* rm_package = (RDB_ROADMARK_t*) Framework::RDBHandler::getFirstEntry(package, RDB_PKG_ID_ROADMARK, num_elements, false);
    while(num_elements && rm_package){
        Lane* l = new Lane(rm_package->id, package->hdr.frameNo);
        unsigned int num_tesselation_points = rm_package->noDataPoints;
        RDB_POINT_t* p = (RDB_POINT_t*)(rm_package + 1);

        while (num_tesselation_points){
            //std::cout << "Tesselation point: [" << p->x << ", " << p->y << ", " << p->z <<"]" << std::endl;
            QVector3D tess_p(p->x, p->y, p->z);
            l->addTesselationPoint(tess_p);
            num_tesselation_points--;
            p++;
        }

        lane_gt.insert(frame_nr, l);

        num_elements--;
        if( num_elements ){
            rm_package = (RDB_ROADMARK_t*)p;
        }
    }
  }

  void VTDSource::convertToDepth(ImageData* image, RDB_CAMERA_t* cameraInfo)
  {
    //z = 0.5*(f+n)/(f-n) + (-f*n)/(f-n) * (1/d) + 0.5
    //
    //with:
    //z: z-buffer value (normalized in [0,1]. Non-normalized fixed point zf = z * (s^n - 1 ) where n is bit depth of the depth buffer)
    //d: distance of fragment (pixel) to xy plane of camera coordinate system
    //n: near plane (camera frustum setting)
    //f: far plane (camera frustum setting)

    unsigned int* src = reinterpret_cast<unsigned int*>(image->ptr);
    float* dst = reinterpret_cast<float*>(image->ptr);

    /*
     *float alpha = cameraInfo->focalY / 2;
     *float n = cameraInfo->height / 2 / tan(alpha);
     */

    for(size_t pos = 0; pos < image->width * image->height; ++pos)
      {
        unsigned int z = src[pos];
        float z1 = (float)z / (unsigned int)0xFFFFFFFF; // ZMAX
        float d = 2 * cameraInfo->clipNear * cameraInfo->clipFar / (cameraInfo->clipFar + cameraInfo->clipNear - z1 * (cameraInfo->clipFar - cameraInfo->clipNear));
        dst[pos] = d;
      }
  }

  void VTDSource::printImageInfo(RDB_IMAGE_t* img)
  {
    std::cout << "Received VTD image number " << img->id << ". Resolution: " << img->width << " x " << img->height << std::endl;
    std::cout << "Pixel format: " << ((unsigned int)img->pixelFormat) << "Pixel size: " << ((unsigned int)img->pixelSize) << std::endl;
    std::cout << "Image size: " << img->imgSize << std::endl;
  }

  // For RGB888 only
  void VTDSource::copyBuffToImageData(RDB_IMAGE_t* buf_img, ImageData& image,
                                      RDB_MSG_t* pRdbMsg) {
    image.width = pRdbMsg->u.image.width;
    image.height = pRdbMsg->u.image.height;
    image.colordepth = 3;
    image.pf = RGB888;
    image.frame_nr = pRdbMsg->hdr.frameNo;
    size_t bufSize = image.width * image.height * 3;
    unsigned char * left_buf = (unsigned char *)std::malloc(sizeof(unsigned char) * bufSize);
    memcpy(left_buf, buf_img, bufSize);
    image.ptr = left_buf;
    ImageUtils::hFlipImageData(image);
  }

  void VTDSource::fillDefaultParameters()
  {
    m_parameters.add("localhost", "VTD host address");
    m_parameters.add(48190, "VTD port");
    m_parameters.add(48195, "Sensor port (GT)");
  }

  void VTDSource::fetchParamValues(){

  }

  void VTDSource::close()
  {
    delete m_vtdConnector;
  }

} /* namespace ImageStreaming */
