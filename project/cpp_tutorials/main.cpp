#include <iostream>
#include <stdlib.h>
#include <bits/unique_ptr.h>
#include "Overload.h"
#include "Larger.h"
#include "Constructor.h"
#include "Template.h"
#include "Virtual.h"
#include <boost/filesystem.hpp>
#include <exception>
#include <opencv2/core/types.hpp>

using namespace cpp_tutorials;

template <typename T>
struct CompareFunctionPtrPairsMine_ {
    typedef bool (*type)(typename std::vector<std::pair<T, T>>::iterator, typename std::vector<std::pair<T, T>>::iterator);
};

template <typename T>
using CompareFunctionPtrPairsMine = bool (*)(typename std::vector<std::pair<T, T>>::iterator, typename std::vector<std::pair<T, T>>::iterator);
typedef bool (*FnPointerUnsigned)(typename std::vector<std::pair<unsigned, unsigned>>::iterator, typename std::vector<std::pair<unsigned, unsigned>>::iterator);


typedef bool (*CompareFunctionPtrPairs)(std::vector<std::pair<float, float>>::iterator, std::vector<std::pair<float, float>>::iterator);

typedef bool (*CompareFunctionPtrPointsInteger)(std::vector<cv::Point2i>::iterator, std::vector<cv::Point2i>::iterator);
typedef bool (*CompareFunctionPtrPointsFloat)(std::vector<cv::Point2f>::iterator, std::vector<cv::Point2f>::iterator);

typedef bool (*CompareFunctionPtrPointsStrange)(std::vector<std::pair<cv::Point2f, cv::Point2f>>::iterator, std::vector<std::pair<cv::Point2f, cv::Point2f>>::iterator);


class MyException : public std::exception {
public:
    const char * what () const throw () {
        return "C++ Exception";
    }
};

template<typename T, int size>
class Array {
public:
    Array(int len=size):len_(len), data_(new T[len]) { } //uniform init

    Array(const Array<T,size>&) {}// copy constructor b(a) or b = a

    Array<T,size>& operator= (const Array<T,size>&) {} //assignment, b = a

    ~Array() {
        delete [] data_;
    }

    unsigned len() const {
        return len_;
    }

    // the constructor looks for non const and if non const is not available, then it looks for corresponding const
    // operator.  (), [] and = cannot be friend / static member functions
    T& operator [] ( int i) {
        if (i > len_) throw;
        return data_[i];
    }

    const T& operator [] ( int i) const {
        if (i > len_) throw;
        return data_[i];
    }

    friend std::ostream& operator<< (std::ostream& out, const Array<T,size> &array) {
        for (int i = 0; i < array.len(); i++) {
            out << "index " << i << " value " << array[i] << ";";
        }
        return out;
    }

private:
    T* data_;
    unsigned len_;
};


class Fraction {
public:
    Fraction(int num = 0, int denom = 1) : num_(num), denom_(denom) {}
    friend std::ostream& operator<< (std::ostream& out, const Fraction& f) {
        out << f.num_ << '/' << f.denom_;
        return out;
    }
    friend Fraction operator* ( const Fraction& f1, const Fraction& f2) {
        Fraction f3(f1.num_*f2.num_, f1.denom_*f2.denom_);
        return f3;   // Lvalue
        //return Fraction(f1.num_*f2.num_, f1.denom_*f2.denom_); // rvalue
    }
    friend Fraction square(Fraction& f ) {
        return Fraction(f.num_*f.num_, f.denom_*f.denom_);
    }
    Fraction square_nofriend(Fraction& f) {
        return Fraction(f.num_*f.num_, f.denom_*f.denom_);
    }


protected:
    int num_, denom_;
};


float Q_rsqrt( float number )
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i = y;
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}

template<typename my_type>
void char_to_type() {

    char  *char_data_  = static_cast<char *>("abcdefghijklmnopqrstuvwx");
    my_type *my_type_data_memcpy = reinterpret_cast<my_type *>(malloc(strlen(char_data_)));

    printf("%ld\n", strlen(char_data_));

    // Copies count bytes from the object pointed to by src to the object pointed to by dest.
    // Both objects are reinterpreted as arrays of unsigned char.
    memcpy(my_type_data_memcpy, (char_data_), strlen(char_data_));
    my_type *my_type_data_simple = reinterpret_cast<my_type *>(char_data_);

    for ( auto n = 0 ; n < strlen(char_data_)/sizeof(my_type); n++) {
        printf("%08X\n", *(my_type_data_simple + n));
    }

    printf("end of simple copy\n");

    for ( auto n = 0 ; n < strlen(char_data_)/sizeof(my_type); n++) {
        printf("%08X\n", *(my_type_data_memcpy + n));
    }

    printf("end of mem copy\n");

}

template <typename T>
inline T const& Max (T const& a, T const& b) {
    return a < b ? b:a;
}



int main ( int argc, char *argv[]) {

    double dist_usk_original = 13.711999433708751;
    float dist_usk_from_vires = (float)13.7119989;
    assert(std::floor((float)(dist_usk_original*1000)) == std::floor(dist_usk_from_vires*1000));


    std::vector<unsigned> check_clear_operation(10);

    std::cout << " " << check_clear_operation.size() << std::endl;

    check_clear_operation.clear();

    std::cout << " " << check_clear_operation.size() << std::endl;

    check_clear_operation.push_back(9);

    for ( auto &x : check_clear_operation ) {
        std::cout << " " << check_clear_operation.size() << " " << x;
    }

    int i = 39;
    int j = 20;
    std::cout << "Max(i, j): " << Max(i, j) << std::endl;

    double f1 = 13.5;
    double f2 = 20.7;
    std::cout << "Max(f1, f2): " << Max(f1, f2) << std::endl;

    std::string s1 = "Hello";
    std::string s2 = "World";
    cout << "Max(s1, s2): " << Max(s1, s2) << std::endl;

    typedef struct something {
        // 0x3f800000
        float a = 120.0f;
        unsigned int b = 0;
    } SOMETHING;

    SOMETHING float_int_conversion_tutorial;

    unsigned int* ui_src = reinterpret_cast<unsigned int *>(&float_int_conversion_tutorial);
    *ui_src = 0x3f800000;
    float val_from_ui_src = *(reinterpret_cast<float *>(ui_src + 0));

    printf( "%f\n", val_from_ui_src);

    float* float_src = reinterpret_cast<float*>(&float_int_conversion_tutorial);
    float val_from_float_src = *(float_src + 0);

    printf( "%f\n", val_from_float_src);

    unsigned int val = 0xfbc0a8fb;
    float float_val = *(reinterpret_cast<float *>( &val ));
    printf("answer - %f\n", float_val);

    float z_normalized = (float)(0xfbc0a8fb) / std::numeric_limits<uint>::max(); // ZMAX

    float nearClip = 0.1; //m_camera_info.clipNear;
    float farClip = 1500; //m_camera_info.clipFar;
    float depth = ((-farClip*nearClip)/(farClip-nearClip))/(z_normalized-0.5f-0.5f*(farClip+nearClip)/(farClip-nearClip));

    /*
    (gdb) x 0x7fffe988134c+sizeof(RDB_IMAGE_t)
    0x7fffe988136c:	0xfa53f8fa
            (gdb) x 0x7fffe988134c+sizeof(RDB_IMAGE_t) + 1200*200
    0x7fffe98bbcec:	0xfbc086fb
            (gdb) x 0x7fffe988134c+sizeof(RDB_IMAGE_t) + 1200*200+200
    0x7fffe98bbdb4:	0xfbc0a8fb
     */

    return 0;

    char_to_type<unsigned>();
    char_to_type<float>();

    std::cout << "end of malloc, calloc" << std::endl;

    printf("long %ld\n", sizeof(float));

    auto dist_a = 0.033503235849331521;
    auto dist_b = 0.033333729126862222;

    assert(std::floor(dist_a*1000)/1000 == std::floor(dist_b*1000)/1000);
    std::cout << std::floor(dist_a*1000)/1000 << " " << std::floor(dist_b*1000)/1000 << std::endl;
    std::cout << std::round(dist_a*1000)/1000 << " " << std::round(dist_b*1000)/1000 << std::endl;


    int a = 200;
    int& b = a;
    int& c = b;
    int  d = b;  // New variable
    printf("%d\n",b) ; // Before change
    c = 400;
    printf("%d\n",b) ; // After changing c
    d = 500;
    printf("%d\n",b) ; // After changing d

    std::cout << "hello" <<  (1%1) << (2%1) << (1%2) << (5%3) << (3%5) << (4%1) << (0%4) << (1%20);

    try {
        throw MyException();
    }catch(MyException& e) {
        std::cout << "MyException caught" << std::endl;
        std::cout << e.what() << std::endl;
    } catch(std::exception& e) {
        //Other errors
    }

    Fraction n(3,8);
    std::cout << "if n is " << n << ", 5*n is " << n*n << '\n';
    std::cout << n.square_nofriend(n) << '\n';
    std::cout << square(n) << '\n';
    typedef std::shared_ptr<int> UniquePtr;
    UniquePtr ptr_unique1(new int());
    UniquePtr ptr_unique2(new int());
    ptr_unique2 = ptr_unique1;
    if ( ptr_unique1 <= ptr_unique2 )
    {
        std::cout << ptr_unique1.operator->() << " " << ptr_unique2.operator->() << '\n';
    }

    Array<int,10> int_array;
    Array<Array<int,2>,2> arr_of_int_array;

    //int_array.operator<<(std::cout, int_array);
    std::cout << int_array << std::endl;
    std::cout << arr_of_int_array << std::endl;
    int_array[9] = 100;
    int_array.operator[](9) = 50; // the function call operator[]() becomes a l value for the referent data_[i]
    // referenced by T&
    std::cout << "manipulate " << int_array << std::endl;

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
    //delete ptr; // char_data_ to deallocate the memory again will also lead to undefined behavior.
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
    catch (std::exception& exception) {
        // Other errors.
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
