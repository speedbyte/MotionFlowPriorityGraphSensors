//
// Created by veikas on 23.07.17.
//

#include "Template.h"

namespace cpp_tutorials {

    template <typename T>
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

        ~Storage() {}

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

}
