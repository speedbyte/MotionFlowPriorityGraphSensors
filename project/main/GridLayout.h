//
// Created by veikas on 09.09.17.
//

#ifndef MAIN_GRIDLAYOUT_H
#define MAIN_GRIDLAYOUT_H



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

#endif //MAIN_GRIDLAYOUT_H
