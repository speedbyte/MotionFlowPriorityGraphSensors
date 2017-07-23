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

#include<vector>

namespace cpp_tutorials {

    struct DateStruct
    {
        int year;
        int month;
        int day;
    };

    class ConstructorTutorial1 {
    public:
        int m_x;
        int m_y;
        void printall(void) {
            std::cout << " " << m_x << " " << m_y << std::endl;
        }
    };

    int *value = new (std::nothrow) int; // value will be set to a null pointer if the integer allocation fails

    class ConstructorTutorial2 {
    private:
        int hidden_variable;
        static int changeable_static_variable; // This is just a declaration. Staic varialbes and functions cannot be defined

    public:
        const static int new_static_variable = 100;
        // the class, because they dont have any this operator.
        int new_non_static_variable = 200;
        std::vector<std::string> container_array_of_string = {"hello", "world"};
        const char *char_ptr = "hello";
        const char *array_of_char_ptr[2] =  {"hello", "world"};
        const char array_of_char_static[2][10] =  {"hello", "world"};
        std::string array_string[2] = {"hello", "world"};

        ConstructorTutorial2() {
            hidden_variable = 555;
        }

        ConstructorTutorial2(int x) {
            hidden_variable = x;
        }

        ConstructorTutorial2(int x, std::vector<std::string> dummy_str, const char *dummy_char_ptr )
        //,const char foo[], std::string bar[2] )
        {
            hidden_variable = x;
            container_array_of_string = dummy_str;
            char_ptr = dummy_char_ptr;
            //array_of_char_static[0] = foo[0]; //static arrays are not assignable.
            //array_string[0] = bar[0];
        }

        void printall() {
            std::cout << hidden_variable << container_array_of_string[0] << char_ptr << array_of_char_ptr << array_of_char_static << array_string << std::endl;
        }
    };

    //static int ConstructorTutorial2::change_static_variable() {return ConstructorTutorial2::changeable_static_variable+1;}
    static void throwing_an_exception(int x);


    static void throwing_an_exception(int x) {
        throw x;
    }
}


#endif //CPP_TUTORIALS_CONSTRUCTOR_H
