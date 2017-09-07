#include <iterator>
#include <iostream>




/**
 *
 * @tparam Iter
 * @param left
 * @param right
 * @param val
 * @return
 * \brief
 * the program returns the floor in O(logn)
 */
template <class Iter>
Iter lower_bound(Iter left,
                 Iter right,
                 typename std::iterator_traits<Iter>::value_type val)
{
    while (left < right) {
        Iter middle = left + (right - left) / 2;
        if (*middle < val)
            left = middle + 1;
        else
            right = middle;
    }
    return left;
}




int main() {
    int a[] = {12, 20, 20, 20, 32, 40, 52};
    int targets[] = {10, 19, 20, 22, 40, 60};

    for (auto i : targets) {
        std::cout << "Target: " << i
                  << ", position: " << lower_bound(std::begin(a), std::end(a), i) - a << "\n";

    }
}