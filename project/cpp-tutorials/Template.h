//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_TEMPLATE_H
#define CPP_TUTORIALS_TEMPLATE_H

/** brief

 Template:
 The keyword template tells the compiler what follows is the list of template parameters. It is advisable also to
 call the functions using the template keyword because it explicitly tells the compiler that the parameters are not
 < or > signs, but indeed a list of template parameters. Within choosing pass by value or pass by reference, pass by
 reference is preferred, because otherwise we will have copies of the objects on the heap. The best is to declare const
 references.

    int (&xFunc)() = funcX; xFunc() is an alias for funcX

 So my first question is what is typename... and also what is the signifigance of ... almost eveywhere in the code?
    template<uint32_t _num, typename... _types>

 The typename ... types is the declaration of a variadic template pack with the name types. A template parameter pack
 can have zero or more template parameters. The other occurrence of for example types... is the expansion of this
 template pack.

 ... is called ellipsis.

 What does this->template do? As far as I have read, templates are not objects ( they dont exist at run time ) , so
 what is the this pointer referring to?

 The template keyword is a disambiguation keyword similar to typename to tell the compiler that you do not want to call
 the comparison operation < but the templated method instead.

 Does the template line above the function subplot() makes sense? Because the template is normally used, so that the
 compiler can replace the function ( subplot ) differently according to the template parameters. But if the template
 parameters are fixed, then whats the use of template at all?

    template<uint32_t _width, uint32_t _height, uint32_t _cols, uint32_t _rows>

 Finally the template line above subplot defines non-type template parameters which could be used in the method
 subplot. This can have various reasons like performance optimizations.

 Template classes, Template functions

 The compiler is smart enough to know it only needs to create one template instance per set of unique type parameters
 (per file). It’s also worth noting that if you create a template function but do not call it, no template instances
 will be created.

     bool operator>(const Cents &c1, const Cents &c2)

     template <typename T> // this is the template parameter declaration
     template <class T>
     class Array { }
     template <>   // for all types
     class Array<int> {}

 optional : template class Array<int>; // Explicitly instantiate template Array<int>
 optional : template class Array<double>; // Explicitly instantiate template Array<double>

 in main() -> Array<int> or Array<double> ...
     friend ostream& operator<< (ostream &out, const Cents &cents)
     {
         out << cents.m_cents << " cents ";
         return out;
     }

     std::cout << average(array3, 4) << '\n';

 Template classes (not template functions) can make use of another kind of template parameter known as an expression
 parameter.

 Functional template specialisation.

 We override one of the templates functions by explictly defining the function again.
 template <> // the following is a template function with no templated parameters

     void Storage<double>::print()
     {
         std::cout << std::scientific << m_value << '\n';
     }

 Class template specialisation

 Class template specializations are treated as completely independent classes, even though they are allocated in the
 same way as the templated class. This means that we can change anything and everything about our specialization class,
 including the way it’s implemented and even the functions it makes public, just as if it were an independent class.

 template <> // the following is a template class with no templated parameters
 class Storage8<bool> // we're specializing Storage8 for bool
 It’s worth noting again that keeping the public interface between your template class and all of the specializations
 identical is generally a good idea, as it makes them easier to use -- however, it’s not strictly necessary.

 template <class T, int size> // size is the expression parameter
 class StaticArray

 Partial template specialization allows us to specialize classes (but not functions!) where some, but not all, of the
 template parameters have been explicitly defined. For our challenge above, the ideal solution would be to have our
 overloaded print function work with StaticArray of type char, but leave the length expression parameter templated so
 it can vary as needed. Partial template specialization allows us to do just that!

 // overload of print() function for partially specialized StaticArray<char, size>
 template <int size> // size is still a templated expression parameter
 void print(StaticArray<char, size> &array) // we're explicitly defining type char here

 Fortunately, partial template specialization offers us a convenient solution. In this case, we’ll use class partial
 template specialization to define a special version of the Storage class that works for pointer values. This class is
 considered partially specialized because we’re telling the compiler that it’s only for use with pointer types, even
 though we haven’t specified the underlying type exactly.

 Generic template function, generic template class.


 Template specialisation.

 To do so, we can use a function template specialization (sometimes called a full or explicit function template
 specialization) to create a specialized version of the print() function for type double

 template <>
 void Storage<double>::print()

 template <>
 Storage<char*>::Storage(char* value)
 {

 Class template specialization

 template <> // the following is a template class with no templated parameters
 class Storage8<bool> // we're specializing Storage8 for bool

 Storage8<bool> boolStorage;
 Storage8<int> intStorage;

 Note that we have kept the publicly exposed interface of both classes the same -- while C++ gives us free reign to
 add, remove, or change functions of Storage8<bool> as we see fit, keeping a consistent interface means the programmer
 can use either class in exactly the same manner.

 Partial Template Specialization

 template <class T, int size> // size is the expression parameter
 class StaticArray

 template <typename T, int size>
 void print(StaticArray<T, size> &array)
 {}

 print(int4); // calls print with the function parameter of type StaticArray<int,4>. So whereever we have StaticArray
 as a class, it will be replaced by the template definition StaticArray<T,size>. However if there is a specialization,
 then the StaticArray is replaced by the specialization i.e StaticArray<char, 14>

 // Override print() for fully specialized StaticArray<char, 14>
 template <>
 void print(StaticArray<char, 14> &array)

 The point StaticArray<char, 14> is a problem because what if the charaacter array is 20 bytes. And hence the partial
 template specialisation comes to rescue.

 // overload of print() function for partially specialized StaticArray<char, size>
 template <int size> // size is still a templated expression parameter
 void print(StaticArray<char, size> &array) // we're explicitly defining type char here

 Note that as of C++14, partial template specialization can only be used with classes, not template functions (functions
 must be fully specialized).

 Once any class is defined via a template, it can only be instantiated using the template <>. Similarly fuciton

 The story is that the compiler tries to find its own parameter type depending upon the parameter list. This is implicit, and its not a bad idea to explicitly provide the parameters type within the corner brackets.

 Example 1

     swap(i,j);          // Short cut
     swap<int,int>(i,j); // Long cut

     template<typename T>
     void swap(T& x, T& y)
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

 It’s worth taking a brief look at how template functions are implemented in C++, because future lessons will build off
 of some of these concepts. It turns out that C++ does not compile the template function directly. Instead, at compile
 time, when the compiler encounters a call to a template function, it replicates the template function and replaces the
 template type parameters with actual types. The function with actual types is called a function template instance.

 Template type parameter, Template expression parameter

 int Array<T>::getValue(void) { return m_length; } . this is how you access a class which is defined as a template..
 A normal class would be simply int Array::getValue(void) { return m_length; }

 Just Array would refer to a non-templated version of a class named Array.
 template class member functions goes in the header.
 template class definition goes in the header.  Basically any lines with contains a keyword template goes in the header
 file. One can also force the compiler to stencil out classes by writing template

     class Array<int> ; // Explicitly instantiate template Array<int>..

 Normally we would simply do Array<int> intArray(12); in the source file.
 template expression parameters. Just replace the words with values.

    std::array<int, 5>
    template<uint32_t _width, uint32_t _height, uint32_t _cols, uint32_t _rows>
    void subplot(Figure* out) {

 Dependant name and types. All template instantiation are dependant because the type of the declaration is not known
 until they are intantiated. Non dependant name and type are those whose type and name of the declarations are known at
 the time of the template declaration.

 */


class Template {

};


#endif //CPP_TUTORIALS_TEMPLATE_H
