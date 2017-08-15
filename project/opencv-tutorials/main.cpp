#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<boost/filesystem.hpp>
#include <opencv2/imgproc.hpp>

//using namespace cv;

std::string uniqueName() {
    auto randchar = []() -> char
    {
        const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(10,0);
    std::generate_n( str.begin(), 8, randchar );
    return str;
}

void convertColor(cv::Mat &in, cv::Mat &out ) {
    cv::cvtColor(in, out, CV_BGR2GRAY);
}

void arrange_images(cv::Mat3b &out, cv::Mat3b A, uint32_t _image_cols_width, uint32_t _image_rows_height, uint32_t
_grid_col, uint32_t _grid_row) {

    cv::Mat3b subfig, tmp;
    cv::Mat X(30, 40, cv::DataType<char>::type);
    cv::Mat image(_grid_col*_image_cols_width*2, _grid_row*_image_rows_height*2,  CV_8UC3, cv::Scalar(0));
    for ( int _index = 0; _index<4; _index++ ){
        const uint32_t _row = _index / _grid_col;
        const uint32_t _col = _index - _row * _grid_col;
        cv::Rect rect;

        cv::Mat3b in1, in2, in3, in4;
        // Top left corner, as well as width and height ( column and row )
        rect = cv::Rect(_col*_image_cols_width,_row*_image_rows_height,_image_cols_width,_image_rows_height);
        //subfig = image;
        subfig = image(rect);

        in1 = A; // in2 = B; in3 = C; in4 = D;

        if(in1.channels() == 1)
            cv::cvtColor(in1,tmp,cv::COLOR_GRAY2BGR);
        else
            tmp = in1;
        std::string nWindow = uniqueName();
        cv::resize(tmp,subfig,subfig.size());
        cv::namedWindow(nWindow, cv::WINDOW_AUTOSIZE);
        cv::imshow(nWindow,subfig);
    }
}

int main ( int argc, char *argv[] ) {

    cv::Mat3b A;
    cv::Mat3b B(A);
    cv::Mat3b C; C = A;
    cv::Mat3b D = A;
    cv::Mat3b out;

    A = cv::imread("../data/image.png", 1);
    if ( !A.data ) {
        std::cout << "here";
        throw;
    }
    convertColor(A,B);

    out = cv::imread("../data/image2.JPG", 1);
    if ( !out.data ) {
        std::cout << "here";
        throw;
    }
    convertColor(C,D);

    arrange_images(out, A, A.cols, A.rows, 2, 2);

    cv::namedWindow("OriginalWindow", CV_WINDOW_FREERATIO);
    cv::namedWindow("GrayWindow", CV_WINDOW_FREERATIO);
    imshow("OriginalWindow", A);
    imshow("GrayWindow", B);

    //cv::imwrite("../data/image2_gray.JPG", B);
    //width = m_mat.cols:
    //height = m_mat.rows;
    cv::waitKey(0);

    return 0;
}

void build_red(const cv::Mat& in, cv::Mat& out) {
    out = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);

    cv::Mat zeros = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);
    cv::Mat tmp;
    in.convertTo(tmp, CV_8UC1);

    std::vector<cv::Mat> ch;
    ch.push_back(zeros);
    ch.push_back(zeros);
    ch.push_back(tmp);

    std::cout << "Using " << ch.size() << " channels" << std::endl;
    merge(ch, out);
} // build_red

void someoperation()
{
    cv::Mat image, image1;
    boost::filesystem::path path = "../data/image.png";
    image = cv::imread(path.string());
    std::cout << "Hello World " << path.string();
    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::imshow("Original", image);
    cv::waitKey(0);
    int colSize = image.cols;

    cv::Mat M(100,100,CV_8UC3);  // create a 100*100 8 bit single channel matrix.
    cv::Mat_<char> M2(100,100);
    cv::Mat_<cv::Vec3b> img(240, 320, cv::Vec3b(0,255,0)); // allocate a 320*240 multi channel color image and fill it
    // with green.

    // Not a row, column but a column, row.
    for ( int i = 0; i < image.cols; i++ ) {
        image1 = cv::Vec3b(255,255,255);  // Draw a diagonal white line.
    }
    cv::namedWindow("Modified", CV_WINDOW_AUTOSIZE);
    cv::imshow("Modified", image1);
    cv::waitKey(0);

}
