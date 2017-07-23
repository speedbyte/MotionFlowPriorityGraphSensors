//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_OPERATOROVERLOAD_H
#define CPP_TUTORIALS_OPERATOROVERLOAD_H

/** brief

 Consider the expression std::cout << Point. If the operator is <<, what are the operands? The left operand is the std::cout object of type std::ostream, and the right operand is the Point class object.

 If you are using a member function instead, the left operand automatically becomes the this object. If you are working on a unary operator, then no parameters needs to be passed.

 Although we can overload operator+(Cents, int) as a member function (as we did above), we can’t overload operator+(int, Cents) as a member function, because int isn’t a class we can add members to.

 Also operator<< cannot be overloaded with member function, because the left operand is std::ostream.

 Compiler implicitly converts an object prefix into a hidden leftmost parameter named *this

 A value that has an integral type or enumeration
A pointer or reference to a class object
A pointer or reference to a function
A pointer or reference to a class member function


 */

class OperatorOverload {

};


#endif //CPP_TUTORIALS_OPERATOROVERLOAD_H
