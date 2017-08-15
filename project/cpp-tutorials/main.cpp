#include <iostream>
#include <stdlib.h>
#include <bits/unique_ptr.h>
#include "Overload.h"
#include "Larger.h"
#include "Constructor.h"
#include "Template.h"
#include "OperatorOverload.h"
#include "Pointers.h"
#include "Virtual.h"
#include <boost/filesystem.hpp>

using namespace cpp_tutorials;
using std::unique_ptr;
using std::cout;

template<typename T, int size>
class Array {
public:
    Array(unsigned len=size):len_(len), data_(new T[len]) { } //uniform init
    Array(const Array<T,size>&) {}// copy constructor b(a) or b = a
    Array<T,size>& operator= (const Array<T,size>&) {} //assignment, b = a
    ~Array() { delete [] data_;}
    unsigned len() const {return len_;}
    const T& operator [] ( unsigned i) const { return data_[i];}
    T& operator [] ( unsigned i) { return data_[i];}
    friend std::ostream& operator<< (std::ostream &out, const Array<T,size> &array) {
        for ( int i = 0; i < array.len() ; i++) {
            out<<"index " << i << " value " << array[i] << ";";
        }
        return out;
    }
private:
    T* data_;
    unsigned len_;
};

int main ( int argc, char *argv[]) {

    Array<int,10> int_array;
    Array<Array<int,2>,2> arr_of_int_array;

    std::cout << int_array << std::endl;
    std::cout << arr_of_int_array << std::endl;

    std::vector
    std::cout<<"Current program is Overload" << std::endl;
    cpp_tutorials::overload::Overload O1("Hello");
    cpp_tutorials::overload::Overload O2("World");
    cpp_tutorials::overload::Overload Result1, Result2;
    O1.printDummy();
    O2.printDummy();
    Result1.printDummy();
    Result1 = O1 + O2;
    std::cout << "Operator + using neither friend nor member function - i.e normal function" << Result1.getString();
    Result2 = O1 - O2;
    std::cout << "Operator - using friend function" << Result2.getString();
    std::cout << "Operator << using friend function" << Result2;

    std::cout<<"Current program is Larger" << std::endl;
    cpp_tutorials::larger::Larger<double,double> L1(22.0214234324);
    cpp_tutorials::larger::Larger<int,double> L2(22.000023);
    cpp_tutorials::larger::Larger<char,double> L3('a');
    L1.printLarger();
    L2.printLarger();
    L2.printLargest();
    L3.printLarger();

    // declare an integer array with room for 12 integers
    Storage<int, 12> intArray;

    // Fill it up in order, then print it backwards
    for (int count = 0; count < 6; ++count)
        intArray[count] = count;
    intArray.printArray();
    // declare a double buffer with room for 4 doubles
    Storage<double, 4> doubleArray;

    for (int count = 0; count < 4; ++count)
        doubleArray[count] = (4 + 0.1*count);
    doubleArray.printArray();

    for (int count = 0; count < intArray.getLength(); ++count)
    {
        intArray[count] = count;
    }
    for (int count = intArray.getLength()-1; count >= 0; --count)
        std::cout << intArray[count] << '\n';

    for (int count = 0; count < doubleArray.getLength(); ++count)
    {
        doubleArray[count] = count + 0.5;
    }
    for (int count = doubleArray.getLength()-1; count >= 0; --count) {
        std::cout << doubleArray[count]  << '\n';
    }

    DateStruct today { 2020, 10, 14 }; // use uniform initialization

    Storage<int,4> intArray1;
    Storage<double,6> doubleArray1;

    // A program to calculate the average of array elements.
    int array1[] = {1,2,3,4,6};
    double array2[] = {1,2,3,4,6};
    double avg(0);
    Cents avg_class(0);

    int x(0),y(0);
    int z_main(0);
    Cents nickel(5), dime(10), bigger(0);
    Cents array3[] = { Cents(5), Cents(10), Cents(15), Cents(100), Cents(10) };
/*    std::vector<std::string> container_array_of_string = {"hello", "world"};
    const char *char_ptr = "hello";
    const char *array_of_char_ptr[2] =  {"hello", "world"};
    const char array_of_char_static[2][10] =  {"hello", "world"};
    std::string array_string[2] = {"hello", "world"};  */

    ConstructorTutorial1 instanceContainerTutorial{12,21};
    ConstructorTutorial1 instance2ContainerTutorial = {10,11};
    ConstructorTutorial2 privateVariablesConstructor; // automatically calls ConstructorTutorial2()
    ConstructorTutorial2 privateVariablesConstructor2(123); // automatically calls ConstructorTutorial2()
    // Constructor Init
    //ConstructorTutorial2 privateVariablesConstructor3(123, {"bye", "dear"}, "dummy");
    // Uniform init
    //ConstructorTutorial2 blahblah {456, 123, {"foor", "baar"}, "dummy"};
    // In C++ 11, we can also have uniform initialisation. No need to call the constructor.
// Uniform initialization of a ContructorTutorial2, calls ContructorTutorial2(int, int) constructor
    ConstructorTutorial2 blahblah {456, {"foor", "baar"}, "dummy"} ;// {"foor", "baar"}, "dummy"};

//, {"hello", "world"}, {"hello", "world"}); // automatically calls ConstructorTutorial2()
// There can also be default values and hence it is not mandatory that class instantiation with one parameter will always
    // have a corresponding contstructor with one parameter. If the second parameter is default in the constrcutor, the
    // instantiaioin will still work.
    instanceContainerTutorial.printall();
    instance2ContainerTutorial.printall();
    privateVariablesConstructor.printall();
    privateVariablesConstructor2.printall();
    //privateVariablesConstructor3.printall();
    blahblah.printall();
    //ConstructorTutorial2::new_static_variable = ConstructorTutorial2::new_static_variable + 1;
    cout << ConstructorTutorial2::new_static_variable<< std::endl;
    // cout << ConstructorTutorial2::new_non_static_variable; This wont work.
    //ConstructorTutorial2::change_static_variable();
    //cout << ConstructorTutorial2::change_static_variable();

    int *ptr = new int; // dynamically allocate an integer
    int *value = new (std::nothrow) int; // ask for an integer's worth of memory
    *ptr = 7; // put a value in that memory location
    *value = 7777;
    std::cout << *ptr << std::endl; // Dereferencing a dangling pointer will cause undefined behavior
    std::cout << *value << std::endl; // Dereferencing a dangling pointer will cause undefined behavior
    delete ptr; // return the memory to the operating system.  ptr is now a dangling pointer.
    delete value;
    std::cout << *ptr << std::endl; // Dereferencing a dangling pointer will cause undefined behavior
    ptr = nullptr;
    value = nullptr;
    //std::cout << *ptr; // Dereferencing a dangling pointer will cause undefined behavior
    //delete ptr; // trying to deallocate the memory again will also lead to undefined behavior.
    std::cout << "Begin Template examples" << std::endl;
    // Declare a non-pointer Storage to show it works

    Storage1<int> myint(3);
    myint.print();

    // Declare a pointer Storage to show it works
    int xy = 7;
    Storage1<int*> myintptr(&xy);


    // If myintptr did a pointer assignment on x,
    // then changing x will change myintptr too
    xy = 9;
    myintptr.print();


    std::unique_ptr<Super> inst1(new Super());
    std::unique_ptr<Super> inst2(new Sub());
    inst1->iAm(2);
    inst2->iAm(3);
    std::cout << "Please enter number 1" << std::endl;
    std::cin >> x;
    std::cout << "Please enter number 2" << std::endl;
    std::cin >> y;


    avg = average(array1, 5);
    std::cout << "average of the int array is " << avg << '\n';
    avg = average(array2, 5);
    std::cout << "average of the double array is " << avg << '\n';
    avg_class = average(array3, 5);
    std::cout << "average of the class array is " << avg_class << '\n';

    bigger = add_two_objects(nickel,dime);
    std::cout << "sum of " << nickel << " and " << dime << " class objects is " << bigger  << '\n';


    z_main = add_two_objects(x,y);
    std::cout<< "sum of two integers is " <<  z_main << " and " << add_two_objects(x,y+1.2) << '\n';
    z_main = add_two_objects(x,(y+2.52424324));
    std::cout<< "sum of two double is " << z_main << " and " << add_two_objects(x,y+2.52424324) << '\n';


    std::cout << "average of the int array is " << avg << '\n';
    std::cout << "average of the double array is " << avg << '\n';
    std::cout << "average of the class array is " << avg_class << '\n';
    std::cout << "sum of " << nickel << " and " << dime << " class objects is " << bigger  << '\n';

    try {
        if ( x <= 0 )
        {
            throw "throwing const char* exception by sending SIGABRT to the process" ;
        }
        std::cout << "The number printed is " << x << std::endl;
    }
    catch (const char* exception){
        std::cerr << "Error in main.cpp : " << exception << std::endl;
    }
    try {
        if ( x < 0 )
        {
            throw x ;
        }
        std::cout << "The number printed is " << x << std::endl;
    }
    catch (int exception){
        std::cerr << "Error in main.cpp : The following value is not allowed :" << exception << std::endl;
    }
    try {
        throwing_an_exception(x);
    }
    catch (...) { // We dont know what would be thrown from throwing_an_exception
        std::cout << "How to throw and catch an exception. \n";
    }
    //boost::filesystem path = ""
    //sserr << sspdir(m_framework_path) << "\n\t\tframework root path is not a valid directory: " << m_framework_path
    //       << ssthrow;


    return 0;

}
