
#include <utility>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define MAX_INDEX 5

struct MyIntersection {

    std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > > m_result;

    template < typename T1, typename T2 >
    T1 find_intersection(T1 __first1, T1 __last1, T2 __first2, T2 __last2, T1 __result)
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
                std::cout << "found intersection" << std::endl;
                ++__first1;
                ++__first2;
                ++__result;
            }
        return __result;
    }

    std::vector<std::pair<cv::Point_<float>, cv::Point_<float> > > getResult() {
        return m_result;
    };

    template <typename T1, typename T2>
    bool __comp(T1 lhs, T2 rhs){
        std::cout << "pair of points" << std::endl;
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

};

/// ---------------------------------------------------------------------------------------------------------------------

template <typename T>
struct pairSort {
    inline bool operator() (typename std::pair<T, T>& lhs, typename std::pair<T, T>& rhs){
        if(lhs.first == rhs.first)
            return (lhs.second < rhs.second);
        else
            return lhs.first < rhs.first;
    }
};

template <typename T>
struct pointSort
{
    inline bool operator() (const cv::Point_<T>& lhs, const cv::Point_<T>& rhs) {
        if(lhs.x == rhs.x)
            return (lhs.y < rhs.y);
        else
            return lhs.x < rhs.x;
    }
};

template <typename T>
void print_coordinates_points(T &coordinates1, T &coordinates2) {
    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co1 " << coordinates1.at(index).x << " " << coordinates1.at(index).y << std::endl;
    }

    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co2 " << coordinates2.at(index).x << " " << coordinates2.at(index).y << std::endl;
    }
}

template <typename T>
void points_mine() {

    std::vector<cv::Point_<T>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5.4, 6.1}, {7, 8}, {5.4, 4.5}};
    coordinates2 = {{1, 2}, {0, 0}, {5.4, 4.5}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pointSort<T>());
    std::sort(coordinates2.begin(), coordinates2.end(), pointSort<T>());

    print_coordinates_points(coordinates1, coordinates2);

    std::vector<cv::Point_<T>> results_coordinates(10);

    MyIntersection myIntersection;
    myIntersection.find_intersection(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin());
    // , &(comparePointsIntersectionExplizit<T>)
    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.x << " " << index.y << std::endl;
    }
}

/// --------------------------------------------------------------------------------------------------------------------

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

template <typename T>
void print_coordinates_points_project(T &coordinates1, T &coordinates2) {
    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co1 " << coordinates1.at(index).first.x << " " << coordinates1.at(index).first.y << " " << coordinates1.at(index).second.x << " " << coordinates1.at(index).second.y << std::endl;
    }

    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co2 " << coordinates2.at(index).first.x << " " << coordinates2.at(index).first.y << " " << coordinates2.at(index).second.x << " " << coordinates2.at(index).second.y << std::endl;
    }
}


template <typename T>
void points_mine_project() {

    std::vector<std::pair<cv::Point_<T>, cv::Point_<T>>> coordinates1(MAX_INDEX);
    std::vector<std::pair<cv::Point_<T>, cv::Point_<T>>> coordinates2(MAX_INDEX);

    coordinates1 = {std::make_pair(cv::Point_<T>(1, 2), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(9, 10), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(5.4, 6.1), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(7, 8), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(5.4, 4.5), cv::Point_<T>(0,0))};
    coordinates2 = {std::make_pair(cv::Point_<T>(1, 2), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(0, 0), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(5.4, 4.5), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(7, 8), cv::Point_<T>(0,0)), std::make_pair(cv::Point_<T>(9, 10), cv::Point_<T>(0,0))};

    std::sort(coordinates1.begin(), coordinates1.end(), PointsSort<T>());
    std::sort(coordinates2.begin(), coordinates2.end(), PointsSort<T>());

    print_coordinates_points_project(coordinates1, coordinates2);

    bool sorted = std::is_sorted(coordinates1.begin(), coordinates1.end(), PointsSort<T>());
    std::cout << "sorted = " << sorted << std::endl;

    std::vector<std::pair<cv::Point_<T>, cv::Point_<T> > > results_coordinates(10);

    MyIntersection myIntersection;
    //myIntersection.__set_intersection_pairs(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &(comparePointsIntersectionExplizitProject<T>));

    myIntersection.find_intersection(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin());

    std::cout << "show intersection" << std::endl;
    for ( const auto index : myIntersection.getResult() )  {
        std::cout << index.first.x << " " << index.first.y << std::endl;
    }

}

/// --------------------------------------------------------------------------------------------------------------------


template <typename T>
void print_coordinates_pair(T &coordinates1, T &coordinates2) {

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).first << " " << coordinates1.at(index).second << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).first << " " << coordinates2.at(index).second << std::endl;
    }

}


template <typename T>
void pairs_mine() {

    std::vector<std::pair<T, T>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5.4, 6.1}, {7, 8}, {5.4, 4.5}};
    coordinates2 = {{1, 2}, {0, 0}, {5.4, 4.5}, {7, 8}, {9, 10}};

    MyIntersection myIntersection;
    std::sort(coordinates1.begin(), coordinates1.end(), pairSort<T>());
    std::sort(coordinates2.begin(), coordinates2.end(), pairSort<T>());

    print_coordinates_pair(coordinates1, coordinates2);

    bool sorted = std::is_sorted(coordinates1.begin(), coordinates1.end());
    std::cout << "sorted = " << sorted << std::endl;

    std::vector<std::pair<T, T>> results_coordinates(10);
    myIntersection.find_intersection(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin());

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates ) {
        std::cout << index.first << " " << index.second << std::endl;
    }
}


// ---------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {

    // single value and pairs ( unsigned, float ... ) is implemented in STL
    std::cout << "-------------------------------" << std::endl;
    //pairs_mine<float>();
    //points_mine<float>();
    points_mine_project<float>();

}
