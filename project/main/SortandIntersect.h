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

private:
    std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > > m_result_pair;
    std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > > m_result_disjoint_pair;
    std::vector<cv::Point_<float>> m_result;

public:
    template < typename T1, typename T2, typename R >
    T1 find_intersection(T1 __first1, T1 __last1, T2 __first2, T2 __last2, R __result)
    {
        while (__first1 != __last1 && __first2 != __last2)
            if (__comp(__first1, __first2))
                ++__first1;
            else if (__comp(__first2, __first1))
                ++__first2;
            else
            {
                *__result = *__first1;
                m_result.push_back(*__result);
                //std::cout << "found intersection" << std::endl;
                ++__first1;
                ++__first2;
                ++__result;
            }
        return __result;
    }

    template < typename T1, typename T2, typename R >
    T1 find_intersection_pair(T1 __first1, T1 __last1, T2 __first2, T2 __last2, R __result)
    {
        while (__first1 != __last1 && __first2 != __last2)
            if (__comp_pair(__first1, __first2))
                ++__first1;
            else if (__comp_pair(__first2, __first1))
                ++__first2;
            else
            {
                *__result = *__first1;
                m_result_pair.push_back(*__result);
                //std::cout << "found intersection" << std::endl;
                ++__first1;
                ++__first2;
                ++__result;
            }
        return __result;
    }

    template < typename T1, typename T2, typename R >
    T1 find_disjoint_pair(T1 __first1, T1 __last1, T2 __first2, T2 __last2, R __result) {
        while (__first1 != __last1 && __first2 != __last2)
            if (__comp_pair(__first1, __first2)) {
                ++__first1;
                //m_result_disjoint_pair.push_back(*__first1);
            }
            else if (__comp_pair(__first2, __first1)) {
                ++__first2;
                m_result_disjoint_pair.push_back(*__first2);
            }
            else
            {
                *__result = *__first1;
                //std::cout << "found intersection" << std::endl;
                ++__first1;
                ++__first2;
                ++__result;
            }
        return __result;
    }


    const std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > >& getResultIntersectingPair() {
        return m_result_pair;
    };

    const std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > >& getResultDisjointPair() {
        return m_result_disjoint_pair;
    };

    const std::vector<cv::Point_<float>>& getResult() {
        return m_result;
    };

    template <typename T1, typename T2>
    bool __comp(T1 lhs, T2 rhs) {
        if((*lhs).x == (*rhs).x) {
            return ((*lhs).y < (*rhs).y);
        }
        else
            return (*lhs).x < (*rhs).x;
    }

    template <typename T1, typename T2>
    bool __comp_pair(T1 lhs, T2 rhs){
        if((*lhs).first.x == (*rhs).first.x) {
            return ((*lhs).first.y < (*rhs).first.y);
        }
        else
            return (*lhs).first.x < (*rhs).first.x;
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

    void showResult() {

        for (ushort index = 0; index < m_result.size(); index++) {
            std::cout << "co1 " << m_result.at(index).x << " " << m_result.at(index).y << std::endl;
        }
    }


};


#endif //MAIN_SORTANDINTERSECT_H
