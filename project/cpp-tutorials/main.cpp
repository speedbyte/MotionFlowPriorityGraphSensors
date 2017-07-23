#include <iostream>
#include "Overload.h"
#include "Larger.h"
#include "Constructor.h"
#include "Template.h"

using namespace cpp_tutorials;

int main ( int argc, char *argv[]) {

    using std::cout;
    DateStruct today { 2020, 10, 14 }; // use uniform initialization

    Array<int> intArray(12);
    Array<double> doubleArray(10);

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

    Storage<int> myint(5);
    myint.print();

    // Declare a pointer Storage to show it works
    int xy = 7;
    Storage<int*> myintptr(&xy);


    // If myintptr did a pointer assignment on x,
    // then changing x will change myintptr too
    xy = 9;
    myintptr.print();


    std::unique_ptr<Super> inst1(new Super());
    std::unique_ptr<Super> inst2(new Sub());
    inst1->iAm();
    inst2->iAm();
    std::cout << "Please enter number 1" << std::endl;
    std::cin >> x;
    std::cout << "Please enter number 2" << std::endl;
    std::cin >> y;

    for (int count = 0; count < intArray.getLength(); ++count)
    {
        intArray[count] = count;
        doubleArray[count] = count + 0.5;
    }
    for (int count = intArray.getLength()-1; count >= 0; --count)
        std::cout << intArray[count] << "\t" << doubleArray[count] << '\n';


    avg = average(array1, 5);
    cout << "average of the int array is " << avg << '\n';
    avg = average(array2, 5);
    cout << "average of the double array is " << avg << '\n';
    avg_class = average(array3, 5);
    cout << "average of the class array is " << avg_class << '\n';

    bigger = add_two_objects(nickel,dime);
    cout << "sum of " << nickel << " and " << dime << " class objects is " << bigger  << '\n';


    z_main = add_two_objects(x,y);
    cout<< "sum of two integers is " <<  z_main << " and " << add_two_objects(x,y+1.2) << '\n';
    z_main = add_two_objects(x,y+2.52424324);
    cout<< "sum of two double is " << z_main << " and " << add_two_objects(x,y+2.52424324) << '\n';


    cout << "average of the int array is " << avg << '\n';
    cout << "average of the double array is " << avg << '\n';
    cout << "average of the class array is " << avg_class << '\n';
    cout << "sum of " << nickel << " and " << dime << " class objects is " << bigger  << '\n';

    try {
        if ( x < 0 )
        {
            throw "throwing const char* exception by sending SIGABRT to the process" ;
        }
        cout << "The number printed is " << x << std::endl;
    }
    catch (const char* exception){
        std::cerr << "Error in main.cpp : " << exception << std::endl;
    }
    try {
        if ( x < 0 )
        {
            throw x ;
        }
        cout << "The number printed is " << x << std::endl;
    }
    catch (int exception){
        std::cerr << "Error in main.cpp : The following value is not allowed :" << exception << std::endl;
    }
    try {
        throwing_an_exception(x);
    }
    catch (...) { // We dont know what would be thrown from throwing_an_exception
        std::cerr << "Error in main.cpp :  undetermined type\n";
    }




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
    StaticArray<int, 6> intArray;

    // Fill it up in order, then print it backwards
    for (int count = 0; count < 6; ++count)
        intArray[count] = count;
    intArray.printArray();
    // declare a double buffer with room for 4 doubles
    StaticArray<double, 4> doubleArray;

    for (int count = 0; count < 4; ++count)
        doubleArray[count] = (4. + 0.1*count);
    doubleArray.printArray();

    return 0;

}
