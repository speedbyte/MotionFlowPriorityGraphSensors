#ifndef VTDCAMERASENSOR_H
#define VTDCAMERASENSOR_H

#include <memory>
#include <QImage>

#include <adsim/vtd/rdb_codec.h>

class ByteImage;
class DepthImage;

/**
 * @brief Sensor to connect to shm and fetch images from VTD
 */
class VTDCameraSensor : public adsim::vtd::RDBCodec
{
public:
	explicit VTDCameraSensor(adsim::vtd::RDBTransceiver* rdb_client);

	void process() override { RDBCodec::process(); }

	std::unique_ptr<ByteImage>* getLeftImage() { return m_left_img; }
	std::unique_ptr<ByteImage>* getRightImage() { return m_right_img; }
	std::unique_ptr<DepthImage>* getDepthImage() { return m_depth_img; }

	/**
	 * @brief Cut part of the left image
	 * @param rect The part to cut from the image
	 * @return Cut of the image
	 */
	QImage getCutFromLeftImage(QRect rect);

	/**
	 * @brief Check if all three images (left, right, depth) were fetched from
	 *        shm.
	 * @return True if all images are ready
	 */
	bool hasData() { return m_left_img && m_right_img && m_depth_img; }

	/**
	 * @brief Reset all pointers; the data gets freed in plugins down the chain
	 */
	void clear() { m_left_img = nullptr; m_right_img = nullptr; m_depth_img = nullptr; }

	const RDB_IMAGE_t& getImageInfo() const { return m_image_info; }
	const RDB_CAMERA_t& getCameraInfo() const { return m_camera_info; }
	void printImageInfo() { Framework::RDBHandler::print(m_image_info); }
	void printCameraInfo() { Framework::RDBHandler::print(m_camera_info); }

protected:
	// overwriting and implementing the process methods to obtain data and store locally
	void process(RDB_CAMERA_t* camera) override;
	void process(RDB_IMAGE_t* image) override;

	// Camera Info
	RDB_CAMERA_t m_camera_info;

	// RDB image information
	RDB_IMAGE_t m_image_info;

	// images
	std::unique_ptr<ByteImage>* m_left_img;
	std::unique_ptr<ByteImage>* m_right_img;
	std::unique_ptr<DepthImage>* m_depth_img;

private:
	/**
	 * @brief Convert the vtd depth image to real depth
	 * @param image depth image to be converted in place
	 */
	void convertToDepth(DepthImage& image);

	/**
	 * @brief Copy char buffer image data to a ByteImage or DepthImage
	 * @param buf_img char buffer containing the image from vtd
	 * @param image Empty but initialized ByteImage or DepthImage
	 */
	template<typename T>
	void copyBuffToImageData(uchar* buf_img, T& image);
};

#endif // VTDCAMERASENSOR_H
