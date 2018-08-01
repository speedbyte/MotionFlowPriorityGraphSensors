//
// Created by veikas on 01.08.18.
//

#ifndef MAIN_SORTANDINTERSECT_H
#define MAIN_SORTANDINTERSECT_H

#include <opencv2/core/types.hpp>

template <typename T>
class PointsSort
{
public:
    inline bool operator() (const std::pair<cv::Point_<T>, cv::Point_<T>>& lhs, const std::pair<cv::Point_<T>, cv::Point_<T>>& rhs) {
        if(lhs.first.x == rhs.first.x)
            return (lhs.first.y < rhs.first.y);
        else
            return lhs.first.x < rhs.first.x;
    }
};


#endif //MAIN_SORTANDINTERSECT_H
