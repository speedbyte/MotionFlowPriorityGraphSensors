
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"
/**
 Are the displacement vectors linear?

 Read two manual image files without rain. The two image files are from Marcel
 Read two manual image files with rain. Converted via Dennis tool.

 Read two kitti image files without rain. The two image files are from kitti
 Read two kitti image files with rain. Converted via Dennis tool.

 Read two VIRES image files without rain. The two image files are from VIRES
 Read two VIRES image files with rain. Converted via Dennis tool.

 For the ground truth, you need an external source.

 Parallel Threads:
 Thread 1:  1. Calculate Optical Flow between the two image file
            2. Ground Truth displacement vector is generated at the time of generating the files using coordinate system.
            3. Compare with ground truth.
            4. Plot Displacement Vector
            5. Standard deviation of all the displacement vector magnitude.
            6. Standard deviation of all the displacement vector angle.
 Thread 2:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is directly from the testing data in kitti database. Kitti guys generated the ground truth
            using Velodyne.
            3. Follow Step 2 onwards from above.
 Thread 3:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is from VIRES coordinate system.
            2. Follow Step 2 onwards from above.
*/

/**
 * A template requires a constant expression for a non-type parameter. For a const variable to be used in a constant expression, its initialization must be visible to the compiler. So you'll probably have to include header1.hpp in header2.hpp.
 * image_manual_bare1’ was not declared ‘constexpr’
 * the value of ‘image_manual_bare1’ is not usable in a constant expression
 */


//template <int row, int col>
class GridLayout {
public:

    GridLayout(const cv::Mat& m1, const cv::Mat& m2):m_upper_image(m1),m_lower_image(m2)  {
        m_row = m_upper_image.rows;
        m_col = m_upper_image.cols;
        m_grid.create(m_row*2,m_col,m_upper_image.type());
    }

    const cv::Mat& render() {

        cv::Rect roi_upper(0,0,m_col,m_row);
        cv::Rect roi_lower(0,m_row,m_col,m_row);

        cv::Mat roi_grid_upper(m_grid, roi_upper);
        cv::Mat roi_grid_lower(m_grid, roi_lower );

        m_upper_image.copyTo(roi_grid_upper);
        m_lower_image.copyTo(roi_grid_lower);

        return m_grid;
    }

private:
    // declaring new cv::Mat headers
    cv::Mat m_upper_image;
    cv::Mat m_lower_image;
    int m_row;
    int m_col;
    cv::Mat m_grid;

};

class ImageShow {
public:
    void show(const cv::Mat& img) {
        cv::namedWindow("Grid", CV_WINDOW_AUTOSIZE);
        cv::imshow("Grid", img);
    }
};

int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path kitti_dataset_path, kitti_image_name1,
            kitti_image_name2, kitti_image_path1, kitti_image_path2;

    kitti_dataset_path += RAW_DATASET_PATH;
    kitti_dataset_path += "image_02/data/";
    kitti_image_name1 += "0000000169.png";
    kitti_image_name2 += "0000000170.png";
    kitti_image_path1 += kitti_dataset_path;
    kitti_image_path1 += kitti_image_name1;
    kitti_image_path2 += kitti_dataset_path;
    kitti_image_path2 += kitti_image_name2;

    try {
        if ( !boost::filesystem::exists(kitti_image_path1) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_image_path1.string() << ":" <<  exception;
        exit(0);
    }

    try {
        if ( !boost::filesystem::exists(kitti_image_path2) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_image_path2.string() << ":" <<  exception;
        exit(0);
    }

    std::cout << kitti_image_path1 << std::endl << kitti_image_path2 << std::endl;;

    cv::Mat image_manual_bare1(cv::imread(kitti_image_path1.string(), CV_LOAD_IMAGE_COLOR));
    cv::Mat image_manual_bare2(cv::imread(kitti_image_path2.string(), CV_LOAD_IMAGE_COLOR));

    GridLayout grid_manual(image_manual_bare1, image_manual_bare2);
    cv::Mat image_manual_bare_grid = grid_manual.render();
    ImageShow grid_manual_show;
    grid_manual_show.show(image_manual_bare_grid);


    cv::waitKey(0);
    cv::destroyAllWindows();

    //cv::cartToPolar(image_manual_bare1, ypts, magnitude, angle);


}

