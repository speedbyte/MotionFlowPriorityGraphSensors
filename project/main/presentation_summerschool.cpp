
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"
/**

 Papers:
 Regression analysis to determine the robustness of optical flow algorithms.
    In this regression between two independent variables such as one image with noise and one image
    without noise should be analysied. The regression methods can be eigenvalues, covariance, least square method etc.
    Through the regression analysis, it would be helpful to understand how the typical value of the dependant
    variable changes when one of the independent variable is varied, while the other independent variables are kept
    fixed.
    Dependant variable - displacement vector every 100ms
    Independant variable - movement of the subject
    Other independant variable - other objects are kept fixed.
    The analysis is to see how the displacement vector varies when the subject moves. And this is the ground truth,
    regression analysis of the displacement vector against the independant variable ( movement of the pedesterian ).

    The statistical regression analysis as said before is done by both the least squqre method and the eigen value
    method. Ofcourse the underlying algorithm to solve the least square and the eigen value is the QR decomposition
    method. And the methods to calculate the Q and R is the Householder Reflections method. The Householder
    Reflection itself is a linear transformation that enables a vector to be reflected through a plane or through
    hyper plane. The result is the Q component and the R component. The R component can also be achieved by reducing
    the original matrix in Hessenberg form in finite number of steps also through Householders algorithms. The
    Hessenberg is yet not a triangular matrix, but almost a triangular matrix. Further reduction of the Hessenberg
    can be achieved through iterative procedures such as shifted QR factorization, sometimes combined with deflation
    steps. It was also possible to reduce the general matrix directly to a triangular matrix, but doing it through
    Hessenberg matrix, often economizes the arithmetic involved in the QR algorithm for eigen value problems.

    It is required to find two images whose variance is an orthogonal matrix.

 Fingerprinting to determine the robustness of optical flow algorithms.
 Computational complexity is the next step to determine how many BigOh's are required to complete the algorithm. The
 time required to solve the problem ( or the space required, or any measure of complexity ) is calculated as a
 funciton of the size of instance. It is interesting to see how the algorithm to find the robustness scale with
 increase in the input size of images.

 Flowchart for testing algorithms with Image Sensors:
 Acquisiton step and noise induction
 Acquire image data -> Induce Noise by some forumula -> New noised image data

 Apply optical flow algorithm step and noise induction
 Acquire new noised image data -> Induce specific movements as noise > New specific movements image data
 Acquire new specific movements image data -> Apply Optical Flow algorithm -> Output displacement vector

 Apply validation step and noise induction
 Acquire displacement vector -> Apply collision algorithm -> Output collision graph
 Compare input collision graph with output collision graph -> Output Error

 --------------------------------------

 Random variable is displacements greater than 2px in a sample space of an image with 100 moving objects. There can be
 no objects moving and hence the random variable as a real valued function has a value 0, because
 none of the objects are moving.

 --------------------------

 A graph to show the dynamicity of the image. Is the image set very dynamic or relatively static?
 Compare the dynamicity of a scene. Intuitively one can achieve a lot. This dynamicity is the fingerprint of the image.
 How the dynamicity of the image can confirm that two dynamics are the same.

 Ofcourse, the experiments cannot be done on all the movements. A reference needs to be set up and anything apart
 from this reference are outliers. Outlying observations can cause problems because they may strongly influence the
 result. OUtliers are detected by searching for the model fitted by the majority of the data. So, if there are
 many models ( areas ) , the model ( area ) with the maximum number of data can be used as the observation sample,
 and other models can be neglected or called outliers.  There are many robust statistics methods that deals with dealing
 the outliers. Robust procedures are applied on data, to detect the outliers.
 Univariate, low dimensional and high dimensional data, such as
 1. estimation of location and scatter,
 2. linear regression,
 3. principal component analysis, and
 4. classification

 Outliers ( observations, that are different from majority ) can be
 1. Errors
 2. recorded under exceptional circumstances
 3. belong to other populations
 Consequently, they do not fit the model well.


 Fingerprinting - are the displacement vector linear?

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
            7. Cross Correlation map between the two different function output in 5 and 6.
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

