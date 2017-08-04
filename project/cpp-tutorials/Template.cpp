//
// Created by veikas on 23.07.17.
//

#include "Template.h"
#include "Virtual.h"
namespace cpp_tutorials {

    template int average<int>(int*, int );
    template double average<double>(double*, int);
    template double const& add_two_objects<int, double>(int const&, double const&);
    template int const& add_two_objects<int, int>(int const&, int const&);
    template Cents average<Cents>(Cents*, int);
    template Cents const& add_two_objects<Cents, Cents>(Cents const&, Cents const&);

    /** brief
     * Anyonmous parameters are also called unnamed parameters. They have no names, and hence
     * useful in areas,.... variable arguement list.
     * @return
     */
    template int advancedTemplate<>()

    /** brief
     * Function template with 2 arguments
     * @tparam T1
     * @tparam T2
     * @param x
     * @param y
     * @return
     */
    // passing all parameters by values
    template<typename T1, typename T2>
    // auto automatically assigns a return type and one does not need to fix in advance.
    //const T1& add_two_objects(const T1& x,const T2& y) { // This will not work
    //const auto add_two_objects(const T1& x,const T2& y) -> decltype(x+y) {
    const T2 &add_two_objects(const T1 &x, const T2 &y) {
        //static decltype(x+y) z;
        static T2 z(0);
        z = x + y;
        return z; // l value
        //return x+y; // r value
    }


    /** brief
     * Function average
     * @tparam T
     * @param array
     * @param length
     * @return
     */
    template<typename T>
    T average(T *array, int length) {
        T sum = 0;
        for (int count = 0; count < length; count++)
            sum += array[count];
        sum /= length;
        return sum;
    }


    /** brief
     * Function specialization class Storage constructor
     * @param value
     */
    template<>
    Storage1<char *>::Storage1(char *value) {
        // Figure out how long the string in value is
        int length = 0;
        while (value[length] != '\0')
            ++length;
        ++length; // +1 to account for null terminator

        // Allocate memory to hold the value string
        m_value = new char[length];

        // Copy the actual value string into the m_value memory we just allocated
        for (int count = 0; count < length; ++count)
            m_value[count] = value[count];
    }

    /** brief
     * Function full specializazion class Storage destructor
     */
    template<>
    Storage1<char *>::~Storage1() {
        delete[] m_value;
    }

    // the below wont work, because partially specializing just a function in the class is not allowed !
    // Either the whole class needs to be partially specialized or another class needs to be inherited with
    // partial specialization.
    //template<int size>
//    void Storage_Base<double, size>::printArray()
//    {
//        for (int i = 0; i < size; i++)
//            std::cout << std::scientific << m_array[i] << " ";
//        std::cout << "\n";
//    }
    }
