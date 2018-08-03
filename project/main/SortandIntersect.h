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

struct MyIntersection {

    template < typename T1, typename T2 >
    T1 find_intersection(T1 __first1, T1 __last1, T2 __first2, T2 __last2, T1 __result)
    {
        while (__first1 != __last1 && __first2 != __last2)
            if (__comp<float>(__first1, __first2))
                ++__first1;
            else if (__comp<float>(__first2, __first1))
                ++__first2;
            else
            {
                *__result = *__first1;
                ++__first1;
                ++__first2;
                ++__result;
            }
        return __result;
    }

    template <typename T>
    bool __comp(typename std::vector<std::pair<cv::Point_<T>, cv::Point_<T>>>::const_iterator lhs, typename std::vector<std::pair<cv::Point_<T>, cv::Point_<T>>>::const_iterator rhs){
        std::cout << "pair of points" << std::endl;
        return ( (*lhs).first.x < (*rhs).first.x) ;
    }

    template<typename T>
    bool __comp(typename std::vector<std::pair<T, T>>::iterator lhs, typename std::vector<std::pair<T, T>>::iterator rhs){
        std::cout << "pair of basic types" << std::endl;
        return ( (*lhs).first < (*rhs).first ) ;
    }

    template<typename T>
    bool __comp(typename std::vector<cv::Point_<T>>::iterator lhs, typename std::vector<cv::Point_<T>>::iterator rhs){
        std::cout << "point types" << std::endl;
        return ( (*lhs).x < (*rhs).x ) ;
    }

};


#endif //MAIN_SORTANDINTERSECT_H
