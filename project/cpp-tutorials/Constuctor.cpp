#include <iostream>
#include <assert.h>
#include <vector>
#include <memory>


class Animal {
public:
    void /*non-virtual*/ move(void) {
        std::cout << "This animal moves in some way" << std::endl;
    }
    virtual void eat(void) = 0;
};

// The class "Animal" may possess a definition for eat() if desired.
class Llama : public Animal {
public:
    // The non virtual function move() is inherited but not overridden
    void eat(void) override {
        std::cout << "Llamas eat grass!" << std::endl;
    }
};

class Super
{
public:
    virtual void iAm(int a) { std::cout << "I'm the super class!\n"; }
    void iAmHello() { std::cout << "I'm not !\n"; }
};

class Sub : public Super
{
public:
    void iAmHello() override  { std::cout << "I'm the subclass!\n"; }
};

class Cents {
private:
    int m_cents;

public:
    Cents(int cents) : m_cents ( cents ){};
    friend int operator + (const Cents &c1, const Cents &c2) {
        return (c1.m_cents + c2.m_cents);
    }
    int getCents(void) {
        return m_cents;
    }
    friend std::ostream& operator<< (std::ostream &out, const Cents &cents)
    {
        out << cents.m_cents << " cents ";
        return out;
    }
    void operator+=(Cents cents)
    {
        m_cents += cents.m_cents;
    }
    void operator/=(int value)
    {
        m_cents /= value;
    }
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
    // In C++ 11, we can also have uniform initialisation. No need to call the constructor.
    ConstructorTutorial2 blablah {5, 3}; // Uniform initialization of a ContructorTutorial2, calls ContructorTutorial2(int, int) constructor

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

template <class T>
class Array {
private:
    int m_length;
    T *m_data;
public:
    Array() {
        m_length = 0;
        m_data = nullptr;
    }

    Array(int length) {
        m_data = new T[length];
        m_length = length;
    }

    ~Array() {
        delete[] m_data;
    }

    void Erase() {
        delete[] m_data;
        m_data = nullptr;
        m_length = 0;
    }

    T& operator[] (int index) {
        assert(index>=0 && index<m_length);
        return m_data[index];
    }

    int getLength(); // templated getLength defined below.

};

template <typename T>
class Storage
{
private:
    T m_value;
public:
    Storage(T value):m_value(value) {}
//    {
//        m_value = value;
//    }

    ~Storage()
    {
    }

    void print()
    {
        std::cout << m_value << '\n';
    }
};

// Partial class specialisation
template <typename T>
class Storage<T*> // this is a partial-specialization of Storage that works with pointer types
{
private:
    T* m_value;
public:
    Storage(T* value) // for pointer type T
    {
        // For pointers, we'll do a deep copy
        m_value = new T(*value); // this copies a single value, not an array
    }

    ~Storage()
    {
        delete m_value; // so we use scalar delete here, not array delete
    }

    void print()
    {
        std::cout << *m_value << '\n';
    }
};

// Specialisation member function. In this case its a constructor.
template <>
Storage<char*>::Storage(char* value)
{
    // Figure out how long the string in value is
    int length=0;
    while (value[length] != '\0')
        ++length;
    ++length; // +1 to account for null terminator

    // Allocate memory to hold the value string
    m_value = new char[length];

    // Copy the actual value string into the m_value memory we just allocated
    for (int count=0; count < length; ++count)
        m_value[count] = value[count];
}

template<>
Storage<char*>::~Storage()
{
    delete[] m_value;
}

template <typename T>  // templated member function of an template class.
int Array<T>::getLength() { return  m_length;}

// passing all parameters by values
template <typename T1, typename T2>
// auto automatically assigns a return type and one does not need to fix in advance.
//const T1& add_two_objects(const T1& x,const T2& y) { // This will not work
//const auto add_two_objects(const T1& x,const T2& y) -> decltype(x+y) {
const T2 &add_two_objects(const T1 &x,const T2 &y) {
    //static decltype(x+y) z;
    static T2 z(0);
    z = x+y;
    return z;
    //return x+y;
}


template <class T>
T average(T *array, int length) {
    T sum = 0;
    for ( int count = 0; count < length ; count ++)
        sum += array[count];
    sum /= length;
    return sum;
}

static void throwing_an_exception(int x) {
    throw x;
}




