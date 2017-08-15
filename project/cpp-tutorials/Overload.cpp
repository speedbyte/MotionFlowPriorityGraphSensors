//
// Created by veikas on 06.07.17.
//

#include "Overload.h"
#include <iostream>
namespace cpp_tutorials {
    namespace overload {

        Overload::Overload(std::string dummy) {
            std::cout << "Constructing...\n";
            m_dummy = dummy;
        }

        void Overload::printDummy() {
            std::cout << m_dummy;
        }

        std::string Overload::getString() const {
            return m_dummy;
        }

        Overload operator- (const Overload &O1, const Overload &O2) {
            return Overload(O1.m_dummy + O2.m_dummy);
        }

        std::ostream& operator<< (std::ostream &out, const Overload &O1) {
            out << O1.m_dummy << std::endl;  // Chaining, The whole chain is going to be put in the out. Its like
            // out = out + m.dummy + endl. ie the left hand operand is send back.
            return out;
        }

        const Overload operator+ (const Overload &O1, const Overload &O2) {
            std::string result = O1.getString() + O2.getString();
            Overload tmp(result);
            return tmp;
        }
    }
}
