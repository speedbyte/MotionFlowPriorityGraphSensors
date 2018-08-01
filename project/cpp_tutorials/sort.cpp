
#include <utility>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define MAX_INDEX 5

template <typename T>
void print_coordinates_pair(T coordinates1, T coordinates2) {

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).first << " " << coordinates1.at(index).second << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).first << " " << coordinates2.at(index).second << std::endl;
    }

}

template <typename T>
void print_coordinates_points(T coordinates1, T coordinates2) {
    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co1 " << coordinates1.at(index).x << " " << coordinates1.at(index).y << std::endl;
    }

    for (ushort index = 0; index < MAX_INDEX; index++) {
        std::cout << "co2 " << coordinates2.at(index).x << " " << coordinates2.at(index).y << std::endl;
    }
}

template <typename T>
struct pointSort
{
    inline bool operator() (const T& lhs, const T& rhs) {
        if(lhs.x == rhs.x)
            return (lhs.y < rhs.y);
        else
            return lhs.x < rhs.x;
    }

    friend bool operator == (const T& lhs, const T& rhs)
    {
        return lhs.x == rhs.x &&
               lhs.y == rhs.y;
    }
};

template <typename T>
int pairSort(typename std::pair<T, T> lhs, typename std::pair<T, T> rhs){
    return lhs < rhs ;
}

typedef bool (*FnPointerUnsigned)(typename std::vector<std::pair<unsigned, unsigned>>::iterator, typename std::vector<std::pair<unsigned, unsigned>>::iterator);

template<typename T>
bool comparePairsIntersectionExplizit(typename std::vector<std::pair<T, T>>::iterator lhs, typename std::vector<std::pair<T, T>>::iterator rhs){
    return ( (*lhs).first < (*rhs).first ) ;
}

template <typename T>
bool comparePointsIntersectionExplizit(typename std::vector<T>::iterator lhs, typename std::vector<T>::iterator rhs){
    return ( (*lhs).x < (*rhs).x) ;
}


struct MyIntersection {

    template < typename T, typename C>
    T __set_intersection_pairs(T __first1, T __last1, T __first2, T __last2, T __result, C __comp)
    {
        while (__first1 != __last1 && __first2 != __last2)
            if (__comp(__first1, __first2))
                ++__first1;
            else if (__comp(__first2, __first1))
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

};


template <typename T>
void points_mine() {

    std::vector<T> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pointSort<T>());
    std::sort(coordinates2.begin(), coordinates2.end(), pointSort<T>());

    print_coordinates_points<std::vector<T>>(coordinates1, coordinates2);

    std::vector<cv::Point2i> results_coordinates(10);

    MyIntersection myIntersection;
    myIntersection.__set_intersection_pairs(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(),
                                            &(comparePointsIntersectionExplizit<T>));
    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.x << " " << index.y << std::endl;
    }
}


template <typename T>
void pairs_mine() {

    std::vector<std::pair<T, T>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5.4, 6.1}, {7, 8}, {5.4, 4.5}};
    coordinates2 = {{1, 2}, {0, 0}, {5.4, 4.5}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pairSort<T>);
    std::sort(coordinates2.begin(), coordinates2.end(), pairSort<T>);

    print_coordinates_pair<std::vector<std::pair<T, T>>>(coordinates1, coordinates2);

    std::vector<std::pair<T, T>> results_coordinates(10);
    MyIntersection myIntersection;
    myIntersection.__set_intersection_pairs(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &(comparePairsIntersectionExplizit<T>) );

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates ) {
        std::cout << index.first << " " << index.second << std::endl;
    }
}


int main(int argc, char *argv[]) {

    // single value and pairs ( unsigned, float ... ) is implemented in STL
    std::cout << "-------------------------------" << std::endl;
    pairs_mine<float>();
    points_mine<cv::Point2i>();

}

/*
 *
 * 5 48
27 94
75 24
98 82
28 99
49 52
30 44
19 53
39 23
69 68
 */