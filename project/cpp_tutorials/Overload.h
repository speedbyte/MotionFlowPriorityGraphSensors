//
// Created by veikas on 06.07.17.
//

#ifndef CPP_TUTORIALS_OVERLOAD_H
#define CPP_TUTORIALS_OVERLOAD_H

#include <iostream>

namespace cpp_tutorials {
    namespace overload {

        class Overload {
        private:
            std::string m_dummy;
        public:
            Overload(std::string dummy = "Garbage");
            void printDummy();
            std::string getString() const;
            friend Overload operator-(const Overload &O1, const Overload &O2);
            friend std::ostream& operator<< (std::ostream &out, const Overload &O1);
        };

        const Overload operator+ (const Overload &O1, const Overload &O2);
    }
}

#endif //CPP_TUTORIALS_OVERLOAD_H
