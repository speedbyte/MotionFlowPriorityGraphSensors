#include "VTDCameraSensor.h"
#include "ImageUtils.h"

using namespace ImageStreaming;

VTDCameraSensor::VTDCameraSensor(adsim::vtd::RDBTransceiver* rdb_client)
    : RDBCodec(*rdb_client)
    , m_left_img(nullptr)
    , m_right_img(nullptr)
    , m_depth_img(nullptr)
{
	// init first camera model, before the correct one is received via rdb
	m_camera_info.clipNear = 0.1;
	m_camera_info.clipFar = 100;
}

void VTDCameraSensor::process(RDB_IMAGE_t* image)
{
	//	std::cout << "image" << std::endl;
	size_t width = image->width;
	size_t height = image->height;

	uchar* buf_img = reinterpret_cast<uchar*>(image + 1);

	if (image->pixelFormat == RDB_PIX_FORMAT_DEPTH32) {
		if (m_depth_img) return;
        m_depth_img = new std::unique_ptr<DepthImage>(new DepthImage(width, height, (unsigned int)1));
		std::unique_ptr<DepthImage>& depthImage = *m_depth_img;
		copyBuffToImageData<DepthImage>(buf_img, *depthImage);
		convertToDepth(*depthImage);
	}

}



void VTDCameraSensor::convertToDepth(DepthImage& image)
{
	//z = 0.5*(f+n)/(f-n) + (-f*n)/(f-n) * (1/d) + 0.5
	//
	//with:
	//z: z-buffer value (normalized in [0,1]. Non-normalized fixed point zf = z * (s^n - 1 ) where n is bit depth of the depth buffer)
	//d: distance of fragment (pixel) to xy plane of camera coordinate system
	//n: near plane (camera frustum setting)
	//f: far plane (camera frustum setting)

	unsigned int* src = reinterpret_cast<unsigned int*>(image.ptr);
	float* dst = reinterpret_cast<float*>(image.ptr);

    float n = m_camera_info.clipNear;
    float f = m_camera_info.clipFar;
	/*
	 *float alpha = cameraInfo->focalY / 2;
	 *float n = cameraInfo->height / 2 / tan(alpha);
	 */

	for(size_t pos = 0; pos < image.width * image.height; ++pos)
	{
		unsigned int z = src[pos];
		float z1 = (float)z / std::numeric_limits<uint>::max(); // ZMAX
        float d = ((-f*n)/(f-n))/(z1-0.5f-0.5f*(f+n)/(f-n));
		dst[pos] = d;
	}
}

