//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_CONSTRUCTOR_H
#define CPP_TUTORIALS_CONSTRUCTOR_H

/** brief

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
