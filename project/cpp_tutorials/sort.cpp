
#include <utility>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define MAX_INDEX 5

template <typename T>
void print_coordinates(T coordinates1, T coordinates2) {

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).first << " " << coordinates1.at(index).second << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).first << " " << coordinates2.at(index).second << std::endl;
    }

}

typedef bool (*CompareFunctionPtrPoints)(std::vector<cv::Point2i>::iterator, std::vector<cv::Point2i>::iterator);

template <typename T>
bool comparePointsIntersection(T lhs, T rhs){
    return ( (*lhs).x < (*rhs).x) ;
}

template <typename T>
bool comparePointsSort(T lhs, T rhs){
    if ( lhs.x == rhs.x )
        return (lhs.y < rhs.y);
    else
        return (lhs.x < rhs.x) ;
}


template <typename T>
T __set_intersection_points(T __first1, T __last1, T __first2, T __last2, T __result, CompareFunctionPtrPoints __comp)
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


void points_mine() {

    std::vector<cv::Point2i> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), comparePointsSort<cv::Point2i>);
    std::sort(coordinates2.begin(), coordinates2.end(), comparePointsSort<cv::Point2i>);

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co1 " << coordinates1.at(index).x << " " << coordinates1.at(index).y << std::endl;
    }

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << "co2 " << coordinates2.at(index).x << " " << coordinates2.at(index).y << std::endl;
    }

    std::vector<cv::Point2i> results_coordinates(10);

    __set_intersection_points<std::vector<cv::Point2i>::iterator>(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePointsIntersection<std::vector<cv::Point2i>::iterator>);
    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.x << " " << index.y << std::endl;
    }

}

void pairs_original() {

    std::vector<std::pair<unsigned, unsigned>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end());
    std::sort(coordinates2.begin(), coordinates2.end());

    print_coordinates<std::vector<std::pair<unsigned, unsigned>>>(coordinates1, coordinates2);

    std::vector<std::pair<unsigned, unsigned>> results_coordinates(10);
    std::set_intersection(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin());

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.first << " " << index.second << std::endl;
    }
}

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
int comparePairsSort(T lhs, T rhs){
    return lhs < rhs ;
}

template <typename T>
bool comparePairsIntersection(T lhs, T rhs){
    return ( (*lhs).first < (*rhs).first ) ;
}

void pairs_mine() {

    std::vector<std::pair<unsigned, unsigned>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), comparePairsSort<std::pair<unsigned, unsigned>>);
    std::sort(coordinates2.begin(), coordinates2.end(), comparePairsSort<std::pair<unsigned, unsigned>>);

    print_coordinates<std::vector<std::pair<unsigned, unsigned>>>(coordinates1, coordinates2);

    std::vector<std::pair<unsigned, unsigned>> results_coordinates(10);
    __set_intersection_pairs<std::vector<std::pair<unsigned, unsigned>>::iterator>(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePairsIntersection<std::vector<std::pair<unsigned, unsigned>>::iterator>);

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.first << " " << index.second << std::endl;
    }

}

void single_values() {

    std::vector<unsigned> x_values(MAX_INDEX), y_values(MAX_INDEX);

    // seed initializer
    std::seed_seq seed({100, 1000});
    // mersenne twister with seed
    std::mt19937 mt(seed);

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {

        x_values.at(index) = (unsigned)mt() % 100;
        y_values.at(index) = (unsigned)mt() % 100;

    };

    std::sort(x_values.begin(), x_values.end());
    std::sort(y_values.begin(), y_values.end());

    for ( ushort index = 0; index < MAX_INDEX; index++ ) {
        std::cout << " x " << x_values.at(index) << " y " << y_values.at(index) << std::endl;
    }

    std::vector<unsigned> result(MAX_INDEX);
    std::set_intersection(x_values.begin(), x_values.end(), y_values.begin(), y_values.end(), result.begin());

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : result )  {
        std::cout << index << std::endl;
    }

}


int main(int argc, char *argv[]) {

    //single_values();
    pairs_original();
    std::cout << "-------------------------------" << std::endl;
    pairs_mine();
    std::cout << "-------------------------------" << std::endl;
    points_mine();
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