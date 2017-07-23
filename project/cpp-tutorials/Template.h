//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_TEMPLATE_H
#define CPP_TUTORIALS_TEMPLATE_H

/** brief Class templates

 * Dependant and non dependant types:

 All template instantiation are dependant because the type of the declaration is not known until they are
 instantiated. Non dependant name and type are those whose type and name of the declarations are known at the time of
 the template declaration.

 Templates declarations are just stencils. Hence C++ does not compile the template function directly. Instead, at
 compile time, when the compiler encounters a call to a template function, it creates the template function out of
 the template declaration and replaces the template type parameters with actual types. The function with actual
 types is called a function template instance.

 The compiler is smart enough to know it only needs to create one template instance per set of unique type parameters
 (per file). It’s also worth noting that if you create a template function but do not call it, no template instances
 will be created.

 * Class template instantiation

     template class Storage<int>; // Explicitly instantiate template Storage<int>. This is used in cases, when the
     definition of one of the member functions in Storage is done outside the header file. Otherwise, the compiler
     would not know about the Storage<int>.

     Storage<int> intArray(12);
     Storage<bool> boolStorage;

 Template class member functions goes in the header. Template class definition goes in the source. Basically any
 lines with contains a keyword template goes in the header file. One can also force the compiler to stencil out
 classes by declaring templates with specific types.

     class Storage<int> ; // Explicitly instantiate template Array<int>..

 Normally we would simply do in the source file.

 Template expression parameters. Just replace the words with values.

     Storage<int, 12> intArray; // this assumes template<typename T, int size>

 A template expression parameter is a special type of parameter that does not substitute for a type, but is instead
 replaceed by a value. An expression parameter can be any of the following:

 A value that has an integral type or enumeration
 A pointer or reference to a class object
 A pointer or reference to a function
 A pointer or reference to a class member function

 Standard Library:
 std::array<int, 5>, the int is a type parameter, and the 5 is an expression parameter!

 * Class Templates

     class Storage {};// non templated class
     Storage::getLength() { something; }

     template <typename T>
     class Storage {

        T array[10];

        void print() {
            for (int count=0; count < 10; ++count)
            std::cout << array[count] << ' ';
        }
     }

     Storage<any type except bool> value;

 Also, note that the name of the templated array class is Storage<T>, not Storage -- Storage would refer to a
 non-templated  version of a class named Storage.
 Just Storage would refer to a non-templated version of a class named Array.

 The precondition to write the following code is that the basic template should be present. One can only specialize
 certain things, when the base is there.

     template <> // the following is a template class with no templated parameters
     class Storage<bool> { // we're specializing Storage for bool

        T array[10];

        void print() {
            for (int count=0; count < 10; ++count)
            std::cout << array[count] << ' ';
        }
     }// Template class specialization

     Storage<bool> value;

     template <typename T, int size> // size is the expression parameter
     class Storage {

        T array[size];

        //Overload of print() function for partially specialized Storage<char, size>
        void print() {
            for (int count=0; count < size; ++count)
            std::cout << array[count] << ' ';
        }

     }// partial template class specialization

     Storage<bool, 12> value;

 * Function Templates

 Non Member functions:
 // Base print()
    template <typename T, int size>
    void print(Storage<T, size> &array)
    {
        for (int count = 0; count < size; ++count)
            std::cout << array[count] << ' ';
    }

  // Override print() for fully specialized Storage<char, 14>
    template <>
    void print(Storage<char, 14> &array)
    {
        for (int count = 0; count < 14; ++count)
            std::cout << array[count];
    }

 // overload of print() function for partially specialized Storage<char, size>

    template <int size> // size is still a templated expression parameter
    void print(Storage<char, size> &array) // we're explicitly defining type char here
    {
        for (int count = 0; count < size; ++count)
            std::cout << array[count];
    }

 If print was a part of the class itself, then a partial specialization of functions would not be possible.
 The only solution would be to create the partial specialization of the class. But this would be highly
 inefficient, because the whole class needs to be redeclared, just for a small change in one of the menber
 function. Fortunately, there is a work around using inheritance.



 * Class templates specialization

 Class template specializations are treated as completely independent classes, even though they are allocated in the
 same way as the templated class. This means that we can change anything and everything about our specialization class,
 including the way it’s implemented and even the functions it makes public, just as if it were an independent class.

 Now, when we declare a class of type Storage<T>, where T is not a bool, we’ll get a version stenciled from the
 generic templated Storage<T> class. When we declare a class of type Storage<bool>, we’ll get the specialized
 version we just created. Partial template specialization offers us a convenient solution. In this case, we’ll use
 class partial template specialization to define a special version of the Storage class that works for pointer
 values. This class is considered partially specialized because we’re telling the compiler that it’s only for use
 with pointer types, even though we haven’t specified the underlying type exactly.

 Note that we have kept the publicly exposed interface of both classes the same -- while C++ gives us free reign to
 add, remove, or change functions of Storage8<bool> as we see fit, keeping a consistent interface means the programmer
 can use either class in exactly the same manner.
 It’s worth noting again that keeping the public interface between your template class and all of the specializations
 identical is generally a good idea, as it makes them easier to use -- however, it’s not strictly necessary.
 Interfaces are the public functions. The implementation can ofcourse differ.


 * Partial template specialization

 Whereever we have Storage as a class, it will be replaced by the template definition Storage<T,size>.
 However if there is a specialization, then the Storage is replaced by the specialization i.e

     Storage<char, 14>

 The point Storage<char, 14> is a problem because what if the character array is 20 bytes. And hence the partial
 template specialisation comes to rescue.

 Partial template specialization allows us to specialize classes (but not functions!) where some, but not all, of the
 template parameters have been explicitly defined. For our challenge above, the ideal solution would be to have our
 overloaded print function work with Storage of type char, but leave the length expression parameter templated so
 it can vary as needed. Partial template specialization allows us to do just that. Template classes (not template
 functions) can make use of another kind of template parameter known as an expression parameter.


 */

/** brief Function templates

 * Function template instantiation

 Once any class is defined via a template, it can only be instantiated using the template <>. Similarly functions.
 The story is that the compiler tries to find its own parameter type depending upon the parameter list. This is
 implicit, and its not a bad idea to explicitly provide the parameters type within the corner brackets.

 * Function templates

     int Storage<T>::getValue(void) { return m_length; } . this is how you define a function which is defined
     as a template..

 A normal class would be simply int Storage::getValue(void) { return m_length; }

 * Function templates specialization

 To do so, we can use a function template specialization (sometimes called a full or explicit function template
 specialization) to create a specialized version of the print() function for type double
 We override one of the templates functions by explictly defining the function again.

     template <> // the following is a template function with no templated parameters, since we are explicitly
     specifying all the type
     void Storage<char*>::print()
     {
         std::cout << m_value << '\n';
     }

     template <> // the following is a template function with no templated parameters, since we are explicitly
     specifying all the type
     void Storage<double>::print()
     {
         std::cout << std::scientific << m_value << '\n';
     }

 // overload of print() function for partially specialized Storage<char, size>
     template <> // we're explicitly defining all the types, and hence template is empty
     void print(Storage<char, 12> &array) { }

 Note that as of C++14, partial template specialization can only be used with classes, not template functions (functions
 must be fully specialized).

 Example 1

     swap(i,j);          // Short cut
     swap<int,int>(i,j); // Long cut

     template<typename T>
     void swap(T& x, T& y) // We havent used const references, because the arguements are intended to be changed.
     {
       T tmp = x;
       x = y;
       y = tmp;
     }

 Example 2

     swap_internal();       //   Short cut:
     swap_internal<int>();  //   Long cut:

     template<typename T>
     void swap_internal()
     {
       T x=10,y=20;
       T tmp = x;
       x = y;
       y = tmp;
     }

 Beware, that the above wont work with member functions. The class is already instantiated with the type and this type
 is further used by the member functions and hence

     classinstance.swap<int>(); wont work.


 */

/** brief

    std::array<int, 5>
    template<uint32_t _width, uint32_t _height, uint32_t _cols, uint32_t _rows>
    void subplot(Figure* out) {

 What is typename... and also what is the signifigance of ... almost everywhere in the code?
    template<uint32_t _num, typename... _types>

 The typename ... types is the declaration of a variadic template pack with the name types. A template parameter pack
 can have zero or more template parameters. The other occurrence of for example types... is the expansion of this
 template pack.

 ... is called ellipsis.

 What does this->template do? As far as I have read, templates are not objects ( they dont exist at run time ) , so
 what is the this pointer referring to?

 The keyword template tells the compiler what follows is the list of template parameters. It is advisable also to
 call the functions using the template keyword because it explicitly tells the compiler that the parameters are not
 < or > signs, but indeed a list of template parameters.
 The template keyword is a disambiguation keyword similar to typename to tell the compiler that you do not want to call
 the comparison operation < but the templated method instead.

 Does the template line above the function fn() makes sense? Because the template is normally used, so that the
 compiler can replace the function ( fn ) differently according to the template parameters. But if the template
 parameters are fixed, then whats the use of template at all?

    template<uint32_t _width, uint32_t _height, uint32_t _cols, uint32_t _rows>

 The template line above fn() defines non-type template parameters which could be used in the method
 fn. This can have various reasons like performance optimizations.

 */
#include<iostream>
#include<assert.h>

using std::cout;

namespace cpp_tutorials {

    template <typename T1, typename T2>
    // auto automatically assigns a return type and one does not need to fix in advance.
    //const T1& add_two_objects(const T1& x,const T2& y) { // This will not work
    //const auto add_two_objects(const T1& x,const T2& y) -> decltype(x+y) {
    const T2 &add_two_objects(const T1 &x,const T2 &y);

    template <typename T>
    T average(T *array, int length);

    /** brief
     * Template Class with expression parameter
     * @tparam T
     * @tparam size
     */
    template <class T, int size> // size is the expression parameter
    class Storage_Base
    {
        protected:
            // The expression parameter controls the size of the array
            T m_array[size];

        private:
            int m_length;
            T *m_data;

        public:
            Storage_Base() {
                m_length = 0;
                m_data = nullptr;
            }

            Storage_Base(int length) {
                m_data = new T[length];
                m_length = length;
            }

            ~Storage_Base() {
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

            T* getArray();

            void printArray()
            {
                for (int i = 0; i < size; i++)
                    std::cout << m_data[i];
                std::cout << "\n";
            }
    };

    /** brief
     * Template class with expression parameter
     * @tparam T
     * @tparam size
     */
    template <typename T, int size> // size is the expression parameter
    class Storage: public Storage_Base<T, size>
    {
        public:
            Storage()
            {

            }
    };

    /** brief
     * Template function for class Storage
     * @tparam T
     * @return
     */
    template <typename T, int size>  // templated member function of an template class.
    int Storage_Base<T,size>::getLength()
    {
        return  m_length;
    }


    template <typename T>
    class Storage1
    {
        private:
            T m_value;
        public:
            Storage1(T value):m_value(value) {}

            ~Storage1() {}

            void print()
            {
                std::cout << m_value << '\n';
            }
    };

    // Partial class specialisation
    template <typename T>
    class Storage1<T*> // this is a partial-specialization of Storage that works with pointer types
    {
        private:
            T* m_value;
        public:
            Storage1(T* value) // for pointer type T
            {
                // For pointers, we'll do a deep copy
                m_value = new T(*value); // this copies a single value, not an array
            }

            ~Storage1()
            {
                delete m_value; // so we use scalar delete here, not array delete
            }

            void print()
            {
                std::cout << *m_value << '\n';
            }
    };

    /** brief
     * Template class with expression parameter. We have to do this,
     * because it is not possible to pick out member functions and override
     * with different templates.
     * @tparam size
     */
    template <int size> // size is the expression parameter
    class Storage<double, size>: public Storage_Base<double, size>
    {
    public:
        void printArray()
        {
            for (int i = 0; i < size; i++)
                std::cout << std::scientific << this->m_array[i] << " ";
            std::cout << "\n";
        }
    };

}

#endif //CPP_TUTORIALS_TEMPLATE_H
