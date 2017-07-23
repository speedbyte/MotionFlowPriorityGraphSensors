//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_FRIEND_H
#define CPP_TUTORIALS_FRIEND_H

/** brief:

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

namespace cpp_tutorials {

    class Friend {

    };

}


#endif //CPP_TUTORIALS_FRIEND_H
