//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_TYPEDEFS_H
#define CPP_TUTORIALS_TYPEDEFS_H

/** brief
 typedef and #define

    typedef int* int_p1;
    int_p1 a, b, c;  // a, b, and c are all int pointers.

    #define int_p2 int*

    int_p2 a, b, c;  // only the first is a pointer!

    typedef int a10[10];
    a10 a, b, c; // create three 10-int arrays
    int a[10], b[10], c[10];
    typedef int (*func_p) (int);
    func_p fp // func_p is a pointer to a function that takes an int and returns an int
    int (*fp)(int)

 Usually, typedefs are the way to go as they are so much less error-prone. In which case you could use using = as well
 but that's personal preference since they're both the same:

 The only difference between both prototypes is the use of either the keyword class or the keyword typename. Its use is
 indistinct, since both expressions have exactly the same meaning and behave exactly the same way.

 #define is a symbolic constant.
 #ifndef CONSTANTS_H
 #define CONSTANTS_H

 // define your own namespace to hold constants
 namespace constants
 {
     const double pi(3.14159);
     const double avogadro(6.0221413e23);
     const double my_gravity(9.2); // m/s^2 -- gravity is light on this planet
     // ... other related constants
 }
 #endif

 using declaration and using directive

 typedef int &ref_to_int; ref_to_int const r = i;*/

class Typedefs {

};


#endif //CPP_TUTORIALS_TYPEDEFS_H
