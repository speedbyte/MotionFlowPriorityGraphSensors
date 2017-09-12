//
// Created by veikas on 07.07.17.
//

//Program to display largest among two numbers using function templates.

#include "Larger.h"
#include <iostream>

namespace cpp_tutorials {
    namespace larger {
// function specialization
        //template<typename T> Larger<T> operator+ (const Larger<T>& lhs, const Larger<T>& rhs);
        //template<typename T> std::ostream& operator<< (std::ostream& o, const Larger<T>& x);

        template class Larger<int,double>;
        template class Larger<double,double>;
        template class Larger<char,double>;


        template <typename T1, typename T2>
        Larger<T1,T2>::Larger(T1 large) {
            std::cout << "Constructing Larger Class";
            m_large = large;
        }

        template <typename T1, typename T2>
        void Larger<T1,T2>::printLarger() {
            std::cout << "The large number is " <<  m_large << std::endl;
            T2 blah = 10;
        }

        template <typename T1, typename T2>
        void Larger<T1,T2>::printLargest() {
            std::cout << "The largest number is " <<  m_large << std::endl;
        }

        template <>
        void Larger<char,double>::printLarger() {
            std::cout << "Ths is a specialisation of the template with value " <<  m_large << std::endl;
        }

        template <typename T1, typename T2>
        Larger<T1,T2> operator+  (const Larger<T1,T2>& lhs, const Larger<T1,T2>& rhs)
        {
            // ...
        }

        template <typename T1, typename T2>
        std::ostream& operator<< (std::ostream& o, const Larger<T1,T2>& x)
        {
            // ...
        }

    }
}
