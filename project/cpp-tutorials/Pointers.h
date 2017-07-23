//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_POINTERS_H
#define CPP_TUTORIALS_POINTERS_H

/** brief
 Pointers

 int val = 10; // val is an integer
 int const val = 10; // val is a constant integer
 const int val = 10; // val is a constant integer
 int *val; // val is a pointer that points to an integer
 int const *val; // val is a pointer that points to a constant integer
 const int *val; // val is a pointer that points to a constant integer
 int * const val; // val is a constant pointer that points to an integer
 const int * const val; // val is a constant pointer pointing to a constant int. Invoke using &val
 const char* const argv[] // argv is an array of constant pointers pointing to a constant char. Invoke using &argv[]
 int const * const val; // val is a constant pointer pointing to a constant int

 int value = 5;
 const int *ptr = &value; // ptr points to a constant integer. This is OK, although value is not a constant integer
 *ptr = 6; // not allowed

 int value = 5;
 int *ptr = &value;
 *ptr = 6 // allowed

 const int value2 = 10;
 int *ptr2 = &value2; // ptr2 points to an integer and is not allowed.

 int value = 5;
 int * const ptr = &value; //ptr is a constant and once initialised cannot be reinitialised.
 ptr = &value2 ; // not allowed
 *ptr = 6 ; // ofcourse this is allowed because the value itself is not a constant.

 Finally , A const pointer to a const value can not be set to point to another address, nor can the value it is
 pointing to be changed through the pointer. :) Hint for readability. Just remember that the type of value the
 pointer points to is always on the far left.

  References:

 Three kinds of references. References to non const values. References to const values often called const references
 and r-value references.

 References are just aliases. Instead of using the variable name, one can simply use the reference name for accessing
 or modifying the variable. Just as addresses can be printed by using the & operator, the references can also access
 their address using the & operator. There is no difference between a variable and the reference to a variable.

    int value = 5;
    int &ref = value;
    int &invalidRef; // references must be initialised to something

    const int value = 5;
    int &ref = value; // Not allowed just like in pointers, otherwise ref can change the value.
    const int &ref = value; // Allowed, this is reference to const values or also called const references.

    int value = 5;
    const int &ref = value; // this is OK, since the reference is placed in the const memory.
    value++; // allowed
    ref++; // not allowed

    const int &ref = 2+3; // allowed

 References ( not const references ) cannot refer to a r-value ( temporary value ) or a const l-value. However,
 Const References can refer to a r-value and a l-value ( both const and non const ) . Hence const references can be
 applied everywhere. References to const values are particularly useful as function parameters because of their
 versatility. A const reference parameter allows you to pass in a non-const l-value argument, a const l-value argument,
 a literal, or the result of an expression.
 In case, the arguements needs to be changed, then pass using non const references, otherwise pass using const
 references. A restriction is that r-values cannot be assigned to any reference.



 In the above code, getName() will return a pointer to C-style string “Alex”. This is okay since “Alex” will not go out
 of scope when getName() terminates, so the caller can still successfully access it.

 The answer is that std::cout makes some assumptions about your intent. If you pass it a non-char pointer, it will
 simply print the contents of that pointer (the address that the pointer is holding). However, if you pass it an object
 of type char* or const char*, it will assume you’re intending to print a string. Consequently, instead of printing the
 pointer’s value, it will print the string being pointed to instead!

 As a general rule,

 Use references in function parameters and return types to define useful and self-documenting interfaces.
 Use pointers to implement algorithms and data structures.
 The major difference between pointer and reference is that pointer ( not const pointer ) can be reassigned during its
 life time and the references cannot.
 int *const p = &i; and int &r = i; is literally the same. One cannot omit the assignment, because it will be a compile
 error. Reason is that the variables r and p would be dangling if it is not assigned to some object.

 Rebinding a reference is not possible and also assigning a const pointer to another object also not possible.
 const pointers can be NULL, but references cannot be NULL. Thats why, the references are mostly used as fucntion
 parameters, because the called can never pass NULL. However, indirectly passing NULL is possible for example storing
 the NULL in a pointer and then passing the pointer object to the reference. Anoter way to pass NULL is when an object
 is deallocated ( for eample the function returns and all the automatic variables disappear ) .

 A qualified name is one that specifies a scope for example std::cout - here cout is a qualified name.




 */


class Pointers {

};


#endif //CPP_TUTORIALS_POINTERS_H
