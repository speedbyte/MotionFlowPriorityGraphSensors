
#include <utility>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define MAX_INDEX 5

template <typename T>
struct pointSort
{
    inline bool operator() (const T& lhs, const T& rhs) {
        if(lhs.x == rhs.x)
            return (lhs.y < rhs.y);
        else
            return lhs.x < rhs.x;
    }
};

template <typename T>
int pairSort(T lhs, T rhs){
    return lhs < rhs ;
}

template <typename T>
void print_coordinates(T coordinates1, T coordinates2) {

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).first << " " << coordinates1.at(index).second << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).first << " " << coordinates2.at(index).second << std::endl;
    }

}

typedef bool (*CompareFunctionPtrPointsInteger)(std::vector<cv::Point2i>::iterator, std::vector<cv::Point2i>::iterator);
typedef bool (*CompareFunctionPtrPointsFloat)(std::vector<cv::Point2f>::iterator, std::vector<cv::Point2f>::iterator);

typedef bool (*CompareFunctionPtrPointsStrange)(std::vector<std::pair<cv::Point2f, cv::Point2f>>::iterator, std::vector<std::pair<cv::Point2f, cv::Point2f>>::iterator);


template <typename T>
bool comparePointsIntersection(T lhs, T rhs){
    return ( (*lhs).x < (*rhs).x) ;
}


template <typename T, typename Compare>
T __set_intersection_points(T __first1, T __last1, T __first2, T __last2, T __result, Compare __comp)
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


void points_mine_integer() {

    std::vector<cv::Point2i> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pointSort<cv::Point2i>());
    std::sort(coordinates2.begin(), coordinates2.end(), pointSort<cv::Point2i>());

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).x << " " << coordinates1.at(index).y << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).x << " " << coordinates2.at(index).y << std::endl;
    }

    std::vector<cv::Point2i> results_coordinates(10);

    __set_intersection_points<std::vector<cv::Point2i>::iterator, CompareFunctionPtrPointsInteger>(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePointsIntersection<std::vector<cv::Point2i>::iterator>);
    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.x << " " << index.y << std::endl;
    }

}

void points_mine_float() {

    std::vector<cv::Point2f> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pointSort<cv::Point2f>());
    std::sort(coordinates2.begin(), coordinates2.end(), pointSort<cv::Point2f>());

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).x << " " << coordinates1.at(index).y << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).x << " " << coordinates2.at(index).y << std::endl;
    }


    std::vector<cv::Point2f> results_coordinates(10);

    __set_intersection_points<std::vector<cv::Point2f>::iterator, CompareFunctionPtrPointsFloat>(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePointsIntersection<std::vector<cv::Point2f>::iterator>);
    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.x << " " << index.y << std::endl;
    }

}

//coordinates1 = {{1, 2}, {9, 10}, {5.4, 6.1}, {7, 8}, {5.4, 4.5}};
//coordinates2 = {{1, 2}, {0, 0}, {5.4, 4.5}, {7, 8}, {9, 10}};


typedef bool (*CompareFunctionPtrPairs)(std::vector<std::pair<unsigned, unsigned>>::iterator, std::vector<std::pair<unsigned, unsigned>>::iterator);

template <typename T>
T __set_intersection_pairs(T __first1, T __last1, T __first2, T __last2, T __result, CompareFunctionPtrPairs __comp)
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

template <typename T>
bool comparePairsIntersection(T lhs, T rhs){
    return ( (*lhs).first < (*rhs).first ) ;
}

void pairs_mine() {

    std::vector<std::pair<unsigned, unsigned>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pairSort<std::pair<unsigned, unsigned>>);
    std::sort(coordinates2.begin(), coordinates2.end(), pairSort<std::pair<unsigned, unsigned>>);

    print_coordinates<std::vector<std::pair<unsigned, unsigned>>>(coordinates1, coordinates2);

    std::vector<std::pair<unsigned, unsigned>> results_coordinates(10);
    __set_intersection_pairs<std::vector<std::pair<unsigned, unsigned>>::iterator>(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePairsIntersection<std::vector<std::pair<unsigned, unsigned>>::iterator>);

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.first << " " << index.second << std::endl;
    }

}


int main(int argc, char *argv[]) {

    // single value and pairs ( unsigend, float ... ) is implemented in STL
    std::cout << "-------------------------------" << std::endl;
    //pairs_mine();
    std::cout << "-------------------------------" << std::endl;
    //points_mine_integer();
    std::cout << "-------------------------------" << std::endl;
    //points_mine_float();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "-------------------------------" << std::endl;

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