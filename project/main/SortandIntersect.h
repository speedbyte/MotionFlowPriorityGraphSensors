//
// Created by veikas on 01.08.18.
//

#ifndef MAIN_SORTANDINTERSECT_H
#define MAIN_SORTANDINTERSECT_H

#include <opencv2/core/types.hpp>

template <typename T>
class PairPointsSort
{
public:
    inline bool operator() (const std::pair<cv::Point_<T>, cv::Point_<T>>& lhs, const std::pair<cv::Point_<T>, cv::Point_<T>>& rhs) {
        if(lhs.first.x == rhs.first.x)
            return (lhs.first.y < rhs.first.y);
        else
            return lhs.first.x < rhs.first.x;
    }
};

template <typename T>
class PointsSort
{
public:
    inline bool operator() (const cv::Point_<T>& lhs, const cv::Point_<T>& rhs) {
        if(lhs.x == rhs.x)
            return (lhs.y < rhs.y);
        else
            return lhs.x < rhs.x;
    }
};


#endif //MAIN_SORTANDINTERSECT_H
