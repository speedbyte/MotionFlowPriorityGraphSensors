//
// Created by veikas on 07.07.17.
//

#ifndef CPP_TUTORIALS_LARGER_H
#define CPP_TUTORIALS_LARGER_H

#include <iostream>

namespace cpp_tutorials {
    namespace larger {

        //The solution is to convince the compiler while it is examining the class body proper that the operator+ and
        // operator<< functions are themselves templates. There are several ways to do this; one simple approach is
        // pre-declare each template friend function above the definition of template class Larger

        // pre-declare the template class itself
        template<typename T1, typename T2> class Larger;
        // Now this is a friend template
        template<typename T1, typename T2> Larger<T1,T2> operator+ (const Larger<T1,T2>& lhs, const Larger<T1,T2>& rhs);
        template<typename T1, typename T2> std::ostream& operator<< (std::ostream& o, const Larger<T1,T2>& x);

        // If I dont do the above:
        // error: declaration of ‘operator+’ as non-function
        // friend Larger<T> operator+ <> (const Larger<T>& lhs, const Larger<T>& rhs);
        // By declaring the above friend template function, one tells the compiler explicitly, that a template function
        // would soon be seen.

        template <typename T1, typename T2>
        class Larger {
        private:
            T1 m_large;
        public:
            Larger(T1 large = 0);
            void printLarger();
            void printLargest();
            friend Larger<T1,T2> operator+ <> (const Larger<T1,T2>& lhs, const Larger<T1,T2>& rhs);
            friend std::ostream& operator<< <> (std::ostream &out, const Larger<T1,T2>& rhs);
        };
    }
}


#endif //CPP_TUTORIALS_LARGER_H
