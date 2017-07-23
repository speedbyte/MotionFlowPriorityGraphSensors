//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_OPERATOROVERLOAD_H
#define CPP_TUTORIALS_OPERATOROVERLOAD_H

/** brief

 bool operator>(const Cents &c1, const Cents &c2)

 Consider the expression std::cout << Point. If the operator is <<, what are the operands? The left operand is the std::cout object of type std::ostream, and the right operand is the Point class object.

 If you are using a member function instead, the left operand automatically becomes the this object. If you are working on a unary operator, then no parameters needs to be passed.

 Although we can overload operator+(Cents, int) as a member function (as we did above), we can’t overload operator+(int, Cents) as a member function, because int isn’t a class we can add members to.

 Also operator<< cannot be overloaded with member function, because the left operand is std::ostream.

 Compiler implicitly converts an object prefix into a hidden leftmost parameter named *this


 friend ostream& operator<< (ostream &out, const Cents &cents)
 {
     out << cents.m_cents << " cents ";
     return out;
 }
 std::cout << average(array3, 4) << '\n';

 Friend function:
 Friend a functions is a breather for programmer because they do not have to define the getters and setters for the
 private variables. A friend function allows to access all the private variables of a class and hence gaining two
 important things - one does not have to define public functions for this class. This avoids any other class inheriting
 the class and using the public functions. The friend function is just like a normal function.  A friend function may
 be either a normal function, or a member function of another class.  To declare a friend function, simply use the
 friend keyword in front of the prototype of the function you wish to be a friend of the class.  It does not matter
 whether you declare the friend function in the private or public section of the class. Note that we have to pass the
 parent class object to the friend function. This is because friend function is not a member function. It does not have
 a *this pointer, nor does it have an parent class object to work with, unless given one. It is also possible to make
 an entire class a friend of another class. This gives all of the members of the friend class access to the private
 members of the other class. Here is an example:

    // Make the reset() function a friend of this class
    friend void reset(Accumulator &accumulator);

 This simply means, that reset() can be called from anyone. If we want to restrict reset to be called by a specific
 class, then NewClass::reset() needs to be forward declared in the ParentClass. Another way is to open the ParentClass
 completely by declaring the whole class as friend i.e

    friend class NewClass;

 in the prototype for the ParentClass. Friending is commonly used when defining overloading operators.

 */

class OperatorOverload {

};


#endif //CPP_TUTORIALS_OPERATOROVERLOAD_H
