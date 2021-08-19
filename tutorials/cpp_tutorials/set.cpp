#include <algorithm>
#include <iterator>
#include <set>

template <class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
std::set<T, CMP, ALLOC>& operator *= (
        std::set<T, CMP, ALLOC> &s1, const std::set<T, CMP, ALLOC> &s2)
{
    auto iter1 = s1.begin();
    for (auto iter2 = s2.begin(); iter1 != s1.end() && iter2 != s2.end();) {
        if (*iter1 < *iter2) iter1 = s1.erase(iter1);
        else {
            if (!(*iter2 < *iter1)) ++iter1;
            ++iter2;
        }
    }
    while (iter1 != s1.end()) iter1 = s1.erase(iter1);
    return s1;
}

template <class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
std::set<T, CMP, ALLOC>& operator += (
        std::set<T, CMP, ALLOC> &s1, const std::set<T, CMP, ALLOC> &s2)
{
    s1.insert(s2.begin(), s2.end());
    return s1;
}


template <class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
std::set<T, CMP, ALLOC> operator * (
        const std::set<T, CMP, ALLOC> &s1, const std::set<T, CMP, ALLOC> &s2)
{
    std::set<T, CMP, ALLOC> s;
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(),
                          std::inserter(s, s.begin()));
    return s;
}

template <class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
std::set<T, CMP, ALLOC> operator + (
        const std::set<T, CMP, ALLOC> &s1, const std::set<T, CMP, ALLOC> &s2)
{
    std::set<T, CMP, ALLOC> s;
    std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(),
                   std::inserter(s, s.begin()));
    return s;
}


#include <iostream>

using namespace std;

template <class T>
ostream& operator << (ostream &out, const set<T> &values)
{
    const char *sep = " ";
    for (const T &value : values) {
        out << sep << value; sep = ", ";
    }
    return out;
}



int main()
{
    set<int> s1 { 1, 2, 3, 4 };
    cout << "s1: {" << s1 << " }" << endl;
    set<int> s2 { 0, 1, 3, 6 };
    cout << "s2: {" << s2 << " }" << endl;
    set<int> s1I = s1;
    s1I *= s2;
    cout << "s1I: {" << s1I << " }" << endl;
    set<int> s2I = s2;
    s2I *= s1;
    cout << "s2I: {" << s2I << " }" << endl;
    set<int> s1U = s1;
    s1U += s2;
    cout << "s1U: {" << s1U << " }" << endl;
    set<int> s2U = s2;
    s2U += s1;
    cout << "s2U: {" << s2U << " }" << endl;
    return 0;
}