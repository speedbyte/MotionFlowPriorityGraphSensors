//
// Created by veikas on 05.08.17.
//

#ifndef CPP_TUTORIALS_INTRODUCTION_H
#define CPP_TUTORIALS_INTRODUCTION_H

/**

 A class is a way to abstract behaviour, not just a way to encapsulate bits. A class interface must make
 sense from the outside. If a detached user expects services to access an attribute, those services should exist.
 One should think OO as behaviour centric and not data centric.

 Dynamic data binding - virtual functions. cpp traverses the last overriden function in the derived class.
 Data encapsulation - interface between the abstraction and the implementation. The data is hidden from the external interface
 Polymorphism - The base type can be assigned the dervied object type.
 Inheritance - Code reusability. It implies kind of. Ford is a kind of a car.
 Composition - It implies part of. Engine is a part of the car. Hence using composition an entire car can be assembled.
 Data hiding - private and protected data and is only accessible via member functions.

 ABC ( Abstract Base Class ) are a key to the real world OO design. Often you need an abstraction, where several
 different data structures and algorithms must coexist. In these cases, build an ABC that defines the interface but
 not the implementation and write the user code using the interface defined by the ABC. The compiler prevents the
 creation of objects of the ABC. You can only instantiate a concrete dervied class. Technically, an ABC is a class
 that has one or more pure virtual member functions.

 Pure virtual member function specified that a member function will exist on every object of a concrete derived
 class, even though there is no way to implement this member function in the base class. In other words, it forces
 derived classes to provide a definition for this member function. virtual void draw() const = 0;

 Private members are accessible by members and friends of the class. Protected members are accessible by members and
 friends of the class and also members and friends of the derived class. Public members are accessible by everyone.

 A publicly derived class is a kind of its base class. A pointer to a derived class is in fact pointing to the base
 class.

 The run time system will automatically invoke the proper member function when it has been overridden by a derived
 class ( dynamic binding )

 When a virtual function is invoked, the code that gets called is selected based on the type of the object, rather
 than being selected based on the type of the reference ( this is called dynamic binding, because hte binding
 between the function call and the code to be executed is resolved at run time ). In contrast, when a non virtual
 function is invoked, the code that gets called is selected based on the type of the pointer ( this is called static
 binding, because the call is resolved at compile-time )

 Dynamic binding causes functions to be called through a pointer ( vtab ), static typing guarantees that there is
 good code on the end of the pointer.

 Virtual Destructor:
 If anyone anywhere needs to delete a derived class object using a base class reference, then the base class
 destructor needs to be virtual. If a base class has any virtuals, then it needs to have a virtual destructor as well
 . If the base class does not have any virtual, then the class designer wasnt planning to use the class as a base
 class. Because virtuals use a virtual table pointer, hence per object space does not vary much if there is one
 virtual function or many virtual functions. Hence it is advisable to use a virtual destructor. Destrutors are called
 when the object does not have any reference or when the destructor is called explicitly using delete keyword. The
 destructors are particularly important for example closing a shared file or unlocking a semaphore. If the base class
 has a virtual destructor as a pure virtual function, it is needed to define the destructor elsewhere, because
 otherwise, there would be a linker error when the derived class is destructed. The reason is that the derived
 destructor automatically calls the base destructor.

 Base* base = new Derived; This means that base is a pointer to the type Base and contains the address of the Derived
 type. This is a valid statement. Base pointers / references can be assigned to their derivatives.

 Virtual Constructor:
 Virtual constructors do not exist. The simple reason is that constructors are used to create objects. They are like
 machine in a factory churning out a product. Hence, at the time of instantiation of the object, the constructors
 needs to be known. Thinking of constructors as member functions attached to an existing object is the wrong mental
 mode, instead think of them as factories that churns out objects. The virtual table is a pointer table that points
 to member functions ( base or derived or derived derived etc. ) . Since constructors are not member functions, it is
 not possible to make them as virutal.

 This pointer:
 This pointer in a class has a type "pointer to the class". If it is invoked in the member function of the class and
 the member function is const, then the this pointer has a type "const pointer to the class"

 Copy constructor:
 Many a times, one would like to simply copy an already instantiated object.
 This can be done by new CopyExample (&objectToBeCopied) or if done within the class itself,
 new CopyExample( *this )
 One can explicitly specify the base class in the derived constructor. But it is not important, as the base class
 constructor will be called implicitly. Derived(int radius = 0 ): Base(), _radius(radius)

 Scope operator:
 The purpose of the scope operator is to bypass the dynamic binding mechanism. Hence a syntax like
 Derived d; d.Base::f() will call the Base virtual function instead of the Dervied virtual function. Its advisable to
 use scope operator instead of the this pointer.

 Always use array class and not array type for passing array of derived objects to the base reference. This is
 because, the array class has further checks about the indices and run time errors such as index out of range can be
 avoided.

 Overloading and overriding behaviour between directly using the dervied pointer and using base pointer pointing to a
 derived object must be the same.

 Incremental programming - bits representation ( data ) and mechanism ( function ) is inherited. Default parameter in
 the virtual member function in the base class should be the same in the virtual member function of the derived class
 . Apparently, the default parameter is not forwarded, but just the code is forwarded.

 #157 Dynamic Typing:
 Switch case and if / else statements leads to "code finds the code" way of coding. This is called dynamic typing.
 The dynamic typing should be avoided and instead dynamic binding should be used. Dynamic binding refers to virtual
 functions. Ofcourse a program cant have a static knowledge of the things that existed before the execution of the
 program, which forces the developers to use the dynamic typing with switch/case statements. However, with a little
 bit of more creativity and design, virtual functions can be used. This also helps to avoid downcasts ( a pointer
 cast used to convert a base pointer into a dervied pointer object ).

     void printItalics(BasePrinterClass &base, const char *s) // global function accepting base or derived objects.
     Depending on the parameter passed ( dynamic typing ) the code is executed.
     {
         switch ( which_derived_object ) // calls non virtual functions
     }
     void printItalics(BasePrinterClass &base, const char *s)  // virtual functions defined inside the class and the function is called depending
     on the object with which it is bound to.
     {
         base.italics(const char *s)  // italics is a virtual function in each derived class
     }

 Macros and function calls
    #define minMacro(i,j)    (i) > (j) ? (i) : (j)
    val = minMacro(get_i(), get_j())

    int minFunction(int i, int j) { return (i) > (j) ? (i) : (j); }
    val = minFunction(int i, int j) { }   // this will give a correct result

 #168 Constructors and Destructors:


 Constructors provides the ability ot think to existing objects. This means first an object is created and then a
 constructor is immediately called, that initialises this object in a meaningful way.
 Destructor shreds the objects after it has died. It takes away the ability to think from the object. Its like
 digging a grave for a person who is already dead. The death of an object can occur in many ways:
 1. temporary block {...} ends
 2. delete is called
 3. main ends where the object was created

 It is worth mentioning that constructors can only call constructors from their immediate parent/base class.
 Consequently, the C constructor could not call or pass parameters to the A constructor directly. The C constructor
 can only call the B constructor (which has the responsibility of calling the A constructor).
 There is an exception in case of diamond shaped multiple inheritance when the the constructor can be called
 by a non immediate derived class, because otherwise, it would not be clear who should call the parent construtor.
 This is one time when the non immediate derived class is allowed to call a non-immediate-parent constructor directly
 . The virtual base class constructor is called before a non virtual base class constructor. That the most derived
 class is responsible for creation of the virtual base class it not only valid in a diamond scenario, but also when
 the most derived class inherits a virtual base class in single inheritance.

 The nutshell is that the virtual base class is never created by the class that inherited the virtual base class,
 but is created by the most derived class.

 Default Constructor:
 If the base constructor is not called explicitly, then the compiler calls the default constructor written in the
 base class. In the below example, the default constructor has two variables, name and age and both are initialised
 to 0. However, if the base object would have been called by two parameters, then the initialisation would have been
 done according the parameters.

 If no base class constructor is specified, the default base class constructor will be used. In that case, if no
 default base class constructor can be found (or created by default), the compiler will display an error.

 Person person1;   // Default constructor will be called
 PPerson person2("Foo", 50)

     Person(std::string name = "", int age = 0)
        : m_name(name), m_age(age )

 It turns out that there are three different ways to overload operators: the member function way, the friend function
 way, and the normal function way. We’ll first show you the friend function way (because it’s more intuitive for most
 binary operators) and the normal function way. In a later lesson, we’ll cover the member function way (and discuss
 when to use each in more detail).
 impossible :     char c = 'Q';
    std::cout << &c;

 const int* ptr
 int *const ptr
 const int *const ptr
 Both the above can be assigned once i.e during initialisation and its a must that a const object is initialised. An
 uniitialised const object is an error.

 If a class is instantiated as const, the member functions can only be accessed it is declared as const. For example
 int getValue const { return mValue } ; If const is not used, then this member function cannot be called.
 Futhermore, any const member function that attempts to change a member variable or call a non-const member function
 will cause a compiler error to occur. For example: void resetValue() const { m_value = 0; } // compile error, const
 functions can't change member variables.

    void resetValue() const { m_value = 0; }
    void resetValue() { m_value = 0; }

 both are valid and depending on if the class object is instantiated as const, corresponding resetValue would be called.


 Uniform initialisation, Direct Initialisation, Copy

 BaseballPlayer(std::string name = "", int age = 0,
        double battingAverage = 0.0, int homeRuns = 0)
        : Person(name, age), // call Person(std::string, int) to initialize these fields
            m_battingAverage(battingAverage), m_homeRuns(homeRuns)


 It is worth mentioning that constructors can only call constructors from their immediate parent/base class.
 Consequently, the C constructor could not call or pass parameters to the A constructor directly. The C constructor
 can only call the B constructor (which has the responsibility of calling the A constructor).

 C++ has a third access specifier that we have yet to talk about because it’s only useful in an inheritance context.
 The protected access specifier allows the class the member belongs to, friends, and derived classes to access the
 member. However, protected members are not accessible from outside the class.

 If a member is protected, then the derived class can directly use it from the base class and the base class does not
 need to put any setters and getters for the variable.
    Base &rBase = derived;
    Base *pBase = &derived;


 Virtual:
 Virtual classes - this term does not exist in C++
 Virtual base class - This is used solely during inheritance. A virtual base class cannot be created in itself. But
 its a name given to the class that is inherited by a derived class which inherits the base class with the keyword
 virtual.
 Abstract class -
 Virtual inheritance
 Virtual functions
 Pure virtual functions
 Abstract class - this does not have any member data and all the member functions are pure virtual functions.

 This seems to create another problem too...what if Foo was virtual, and B and C had different implementations of
 Foo? What the heck is D supposed to do when it calls Foo? Is there an actual use case where making A inherited
 virtually solves anything? I mean there must be if this was adopted into the standard, right?


 Inheritance:

 When BaseballPlayer inherits from Person, BaseballPlayer acquires the member functions and variables from Person.
 Additionally, BaseballPlayer defines two members of its own: m_battingAverage and m_homeRuns. This makes sense,
 since these properties are specific to a BaseballPlayer, not to any Person.

 Thus, BaseballPlayer objects will have 4 member variables: m_battingAverage and m_homeRuns from BaseballPlayer, and
 m_name and m_age from Person.

 Inheriting from a base class means we don’t have to redefine the information from the base class in our derived
 classes. We automatically receive the member functions and member variables of the base class through inheritance,
 and then simply add the additional functions or member variables we want. Because Derived inherits functions and
 variables from Base, you may assume that the members of Base are copied into Derived. However, this is not true.
 Instead, we can consider Derived as a two part class: one part Derived, and one part Base. When C++ constructs
 derived objects, it does so in phases. First, the most-base class (at the top of the inheritance tree) is
 constructed first. Then each child class is constructed in order, until the most-child class (at the bottom of the
 inheritance tree) is constructed last. This makes sense: logically, a child can not exist without a parent. It’s
 also the safe way to do things: the child class often uses variables and functions from the parent, but the parent
 class knows nothing about the child. Instantiating the parent class first ensures those variables are already
 initialized by the time the derived class is created and ready to use them.

     Derived(double cost=0.0, int id=0)
         : Base(id), // Call Base(int) constructor with value id!
             m_cost(cost) // Derived member_data

 Note that it doesn’t matter where in the Derived constructor initialization list the Base constructor is called --
 it will always execute first.

 Private members can only be accessed by member functions of the same class or friends. This means derived classes
 can not access private members of the base class directly!

 That gives us 9 combinations: 3 member access specifiers (public, private, and protected), and 3 inheritance types
 (public, private, and protected). Private inheritance makes the base variables private in the derived class.
 Protected inheritance makes the base variables protected in the derived class. Public inheritance does not change
 the access specifier of the inherited variables. In all the cases, private base variables are never accessible,
 irrespective of the type of inheritance.

 When derived.identify() is called, the compiler looks to see if function identify() has been defined in the Derived
 class. It hasn’t. Then it starts looking in the inherited classes (which in this case is Base). Base has defined an
 identify() function, so it uses that one. In other words, Base::identify() was used because Derived::identify()
 doesn’t exist.

 Multiple Inheritance:
 Let’s say we wanted to write a program to keep track of a bunch of teachers. A teacher is a person. However, a
 teacher is also an employee (they are their own employer if working for themselves). Multiple inheritance can be
 used to create a Teacher class that inherits properties from both Person and Employee. To use multiple inheritance,
 simply specify each base class (just like in single inheritance), separated by a comma.
 Ambiguity can result when multiple base classes contain a function with the same name.

 When WirelessAdaptor.getID() is compiled, the compiler looks to see if WirelessAdapter contains a function named
 getID() . It doesn’t. The compiler then looks to see if any of the parent classes have a function named getID(). See
 the problem here? The problem is that WirelessAdaptor actually contains TWO getID() functions: one inherited from
 USBDevice, and one inherited from NetworkDevice. Consequently, this function call is ambiguous, and you will
 receive a compiler error if you try to compile it.

 Solution 1. you can explicitly specify which version you meant to call: USBDevice::getID()

 most of the problems that can be solved using multiple inheritance can be solved using single inheritance as well.
 While most of the issues can be addressed through explicit scoping, the maintenance overhead added to your classes
 in order to deal with the added complexity can cause development time to skyrocket.  Many relatively modern
 languages such as Java and C# restrict classes to single inheritance of normal classes, but allow multiple
 inheritance of interface classes. Many object-oriented languages (eg. Smalltalk, PHP) do not even support multiple
 inheritance.
 The iostream library objects std::cin and std::cout are both implemented using multiple inheritance.

 A virtual base class is always considered a direct base of its most derived class (which is why the most derived
 class is responsible for its construction). But classes inheriting the virtual base still need access to it. So in
 order to facilitate this, the compiler creates a virtual table for each class directly inheriting the virtual class
 (Derived1 and Derived2). These virtual tables point to the functions in the most derived class ( MostDerived ).
 Because  the derived classes have a virtual table, that also means they are now larger by a pointer (to the virtual
 table).


 Friend:

 There’s one bit of trickiness that we can run into when trying to call friend functions in base classes, such as
 operator<<. Because friend functions of the base class aren’t actually part of the base class, using the scope
 resolution qualifier won’t work. Instead, we need a way to make our Derived class temporarily look like the Base
 class so that the right version of the function can be called.

 Virtual Function:

 The concept of virtual functions is an important part of polymorphism. The whole idea is based on the fact that in
 object-oriented programming, when a derived class inherits from a base class, an object of the derived class may be
 referred to via a pointer or reference of the base class type instead of the derived class type. The derived object
 is referred via a pointer or reference of the base class and this gives certain advantages to the programmers.

 Virtual functions are resolved late i.e the binding is not yet known at the compile time. In a normal function
 ( without virtual ) the binding is early, because the compiler already knows that member function would be called
 and hence the call to the address is already designated. Virtual functions allow a program to call methods that
 don't necessarily even exist at the moment the code is compiled. It could be possible that the there is a
 overriden function in a derived class in one of the library that is linked at linking stage or an overriden
 function comes from a run time library.

 The modifier ( virtual ) is inherited by all implementations of that method in derived classes, and hence it is not
 strict to prepend the modifier again in all the overriden derived member functions. Only the most base class
 function needs to be tagged as virtual for all of the derived functions to work virtually. However, having the
 keyword virtual on the derived functions does not hurt, and it serves as a useful reminder that the function is a
 virtual function rather than a normal one. Consequently, it’s generally a good idea to use the virtual keyword for
 virtualized functions in derived classes even though it’s not strictly necessary.

 Since most of the time you’ll want your functions to be virtual, why not just make all functions
 virtual? The answer is because it’s inefficient – resolving a virtual function call takes longer than resolving a
 regular one. Furthermore, the compiler also has to allocate an extra pointer for each class object that has one or
 more virtual functions. We’ll talk about this more in future lessons in this chapter.

 A virtual function is a special type of function that, when called, resolves to the most-derived version of the
 function that exists between the base and derived class. This capability is known as polymorphism. A derived
 function is considered a match if it has the same signature (name, parameter types, and whether it is const) and
 return type as the base version of the function. Such functions are called overrides. It is not mandatory to add
 virtual keyword everytime in the derived version. It is enough to write it once in the base class. However, it is
 good coding guideline to mention virtual in the derived member functions too.

 Normally, the base reference to a derived class ( Base &rBase = derived ) can only call a base member function.
 But, if we define the base member functions as virtual, then it looks in the derived part and calls the most
 derived function. This works because every class maintains internally a virtual table. Hence, it takes a little
 bit more time, if a virtual function is called. This also infers that a constructor or destructor cannot call a
 virtual function. This is because, the derived constructor is only called after the base constructor. If there
 is any function call in the base constructor that is virtual, then it would go on to look for the derived version.
 However, the derived version is not yet ready, because the derived constructor is not yet been called. Always make
 your destructor virtual when you are dealing with inheritance.

 Pure Virtual Functions: Pure virtual functions are functions that makes a class abstract. A pure virtual function
 simply acts as a placeholder that is meant to be redefined by derived classes. An pure virtual class cannot be
 instantiated. It can only be derived. Another method to avoid instantiation of a class is by making the constructor
 private or protected. However, the theory of pure virtual function is that the class has got no constructor and
 hence cannot be instantiated at all even from a derived class. Pure virtual functions are just prototypes. The
 derived class is responsible to implement “all” functions prototyped in the base abstract class ( also called pure
 virtual class or an interface class ) and if the derived class does not implement any of the function, then a
 compile error is thrown. The body of a pure virtual function has to be made outside and should not be inline.

 Interface class: Interface class is one example of a pure virtual class. It is just a collection of pure virtual
 functions. It is not allowed to have non abstract member functions in an interface class. It is also not allowed to
 have member variables. Due to its popularity, some languages such as java has defined interface keyword, so that
 the class is explicitly defined as interface, without makiing all the member function as abstract.

 Override and Final: Override is an identifier and tells that the functions are not native implementations, but has
 been derived from a base class. This prevents typos and the programmer would get an compile error, if he was
 willing to create a derived version of a function call, but actually missed something ( for example the name of
 the function was slightly different ). The compiler will complain then that this type of function does not exist
 in the base class. If final is used after override, no more override is allowed. An overridden function with no
 virtual counterpart in the base class does not make any sense and the compiler will throw an error - Error:
 marked ‘override’, but does not override.

 Pointers

 int val = 10; // val is an integer
 int const val = 10; // val is a constant integer
 const int val = 10; // val is a constant integer .. this versin is preferred. leftmost keyword defines the variable
 int *val; // val is a pointer that points to an integer
 int const *val; // val is a pointer that points to a constant integer
 const int *val; // val is a pointer that points to a constant integer
 int * const val; // val is a constant pointer that points to an integer
 const int * const val; // val is a constant pointer pointing to a constant int. Invoke using &val
 const char* const argv[] // argv is an array of constant pointers. The pointers point to a constant char. Invoke using &argv[]
 int const * const val; // val is a constant pointer pointing to a constant int


 void const_fn (int a, int b ) const {  }
 void non_const_fn (int a, int b )  {  }

 const Base &base;
 So, as we have already learnt, the compiler won’t let us call any functions that do not have the "const" keyword.
 base->non_cost_fn(); // error


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
 const References can refer to a r-value and a l-value ( both const and non const ) . Hence const references can be
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

 Within choosing pass by value or pass by reference, pass by
 reference is preferred, because otherwise we will have copies of the objects on the heap. The best is to declare const
 references.

    int (&xFunc)() = funcX; xFunc() is an alias for funcX

#endif //CPP_TUTORIALS_INTRODUCTION_H
