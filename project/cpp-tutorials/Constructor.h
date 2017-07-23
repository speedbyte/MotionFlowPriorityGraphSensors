//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_CONSTRUCTOR_H
#define CPP_TUTORIALS_CONSTRUCTOR_H

/** brief
 It turns out that there are three different ways to overload operators: the member function way, the friend function way, and the normal function way. We’ll first show you the friend function way (because it’s more intuitive for most binary operators) and the normal function way. In a later lesson, we’ll cover the member function way (and discuss when to use each in more detail).
impossible :     char c = 'Q';
    std::cout << &c;

const int* ptr
int *const ptr
const int *const ptr
Both the above can be assigned once i.e during initialisation and its a must that a const object is initialised. An uniitialised const object is an error.

If a class is instantiated as const, the member functions can only be accessed it is declared as const. For example int getValue const { return mValue } ; If const is not used, then this member function cannot be called.
Futhermore, any const member function that attempts to change a member variable or call a non-const member function will cause a compiler error to occur. For example: void resetValue() const { m_value = 0; } // compile error, const functions can't change member variables.

    void resetValue() const { m_value = 0; }

    void resetValue() { m_value = 0; }

both are valid and depending on if the class object is instantiated as const, corresponding resetValue would be called.


Uniform initialisation, Direct Initialisation, Copy

 BaseballPlayer(std::string name = "", int age = 0,
        double battingAverage = 0.0, int homeRuns = 0)
        : Person(name, age), // call Person(std::string, int) to initialize these fields
            m_battingAverage(battingAverage), m_homeRuns(homeRuns)


It is worth mentioning that constructors can only call constructors from their immediate parent/base class. Consequently, the C constructor could not call or pass parameters to the A constructor directly. The C constructor can only call the B constructor (which has the responsibility of calling the A constructor).

C++ has a third access specifier that we have yet to talk about because it’s only useful in an inheritance context. The protected access specifier allows the class the member belongs to, friends, and derived classes to access the member. However, protected members are not accessible from outside the class.

If a member is protected, then the derived class can directly use it from the base class and the base class does not need to put any setters and getters for the variable.
    Base &rBase = derived;
    Base *pBase = &derived;

 */
struct DateStruct
{
    int year;
    int month;
    int day;
};

#endif //CPP_TUTORIALS_CONSTRUCTOR_H
