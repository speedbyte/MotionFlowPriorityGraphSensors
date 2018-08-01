
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

template <typename T>
int pairSort(T lhs, T rhs){
    return lhs < rhs ;
}

typedef bool (*FnPointerUnsigned)(typename std::vector<std::pair<unsigned, unsigned>>::iterator, typename std::vector<std::pair<unsigned, unsigned>>::iterator);

//coordinates1 = {{1, 2}, {9, 10}, {5.4, 6.1}, {7, 8}, {5.4, 4.5}};
//coordinates2 = {{1, 2}, {0, 0}, {5.4, 4.5}, {7, 8}, {9, 10}};


bool comparePairsIntersectionExplizit(std::vector<std::pair<unsigned, unsigned>>::iterator lhs, std::vector<std::pair<unsigned, unsigned>>::iterator rhs){
    return ( (*lhs).first < (*rhs).first) ;
}


struct MyIntersection {

    template < typename T>
    T __set_intersection_pairs(T __first1, T __last1, T __first2, T __last2, T __result, FnPointerUnsigned __comp)
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


template <typename T, typename C>
void pairs_mine() {

    std::vector<std::pair<T, T>> coordinates1(MAX_INDEX), coordinates2(MAX_INDEX);

    coordinates1 = {{1, 2}, {9, 10}, {5, 6}, {7, 8}, {5, 4}};
    coordinates2 = {{1, 2}, {0, 0}, {5, 4}, {7, 8}, {9, 10}};

    std::sort(coordinates1.begin(), coordinates1.end(), pairSort<std::pair<T, T>>);
    std::sort(coordinates2.begin(), coordinates2.end(), pairSort<std::pair<T, T>>);

    print_coordinates<std::vector<std::pair<T, T>>>(coordinates1, coordinates2);

    std::vector<std::pair<T, T>> results_coordinates(10);
    MyIntersection myIntersection;
    myIntersection.__set_intersection_pairs(coordinates1.begin(), coordinates1.end(), coordinates2.begin(), coordinates2.end(), results_coordinates.begin(), &comparePairsIntersectionExplizit );

    std::cout << "begin intersection" << std::endl;
    for ( const auto index : results_coordinates )  {
        std::cout << index.first << " " << index.second << std::endl;
    }


}


int main(int argc, char *argv[]) {

    // single value and pairs ( unsigend, float ... ) is implemented in STL
    std::cout << "-------------------------------" << std::endl;
    pairs_mine<unsigned, std::vector<std::pair<unsigned, unsigned>>::iterator>();

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