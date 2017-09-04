//
// Created by veikas on 05.08.17.
//

#ifndef CPP_TUTORIALS_INTRODUCTION_H
#define CPP_TUTORIALS_INTRODUCTION_H

/**
 Literature:
 Standard Template Library ( Scott Meyers )
 Learncpp.com
 C++ FAQ ( Cline, Molow )
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

 The sequence of the destructor is exactly the opposite of the constructor. First the derived class object is
 destructed and in the reverse sequence as the member data are declared in the class body. Then the non virtual base
 class object is destructed. For virtual base class, the destructor of the base class is only called, when the most
 derived object is destructed. Hint - diamond shaped multiple inheritance.


 This pointer:
 This pointer in a class has a type "pointer to the class". If it is invoked in the member function of the class and
 the member function is const, then the this pointer has a type "const pointer to the class"

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
 cast used to convert a base pointer into a dervied pointer object ). -
 Example downcasting: (Derived&)baseObject.memberFn();

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

 One can explicitly specify the base class in the derived constructor. But it is not important, as the base class
 constructor will be called implicitly. Derived(int radius = 0 ): Base(), _radius(radius)

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

 Virtual Constructor:
 Virtual constructors do not exist. The simple reason is that constructors are used to create objects. They are like
 machine in a factory churning out a product. Hence, at the time of instantiation of the object, the constructors
 needs to be known. Thinking of constructors as member functions attached to an existing object is the wrong mental
 mode, instead think of them as factories that churns out objects. The virtual table is a pointer table that points
 to member functions ( base or derived or derived derived etc. ) . Since constructors are not member functions, it is
 not possible to make them as virutal.
 The Big Three:

 Destructors, Copy constructor and the assignment operator are the big threes. When the program does not explicitly
 defines it, then the compiler synthesises them in the background. So, X a; and X b = a; and X b; b = a; are all
 valid statements irrespective of if the class has defined them explictly. The first one calls a default constructor,
 the second one calls the copy constructor and the third one calls the assignment operator. When a class uses a plain
 T* to implement remote ownership, forgetting any of the Big Three will lead to generation of the wrong code. The
 compiler synthesises constructors and assignment operators automatically if it doesnt find one. Therefore it is
 better to use a managed pointer. Assuming a HeapPtr<Class> ptr; declared inside a class X(). Normally, the compiler
 will try to synthesize this ptr via copy constructor or assignment operator. However, when it sees, that the ptr is
 not a raw pointer but a managed pointer and that the class that is managing the pointer already has a copy
 constructor / assignment operator for the pointer, then it will silently not do anything. Had this been a raw
 pointer, like Class *ptr, then the compiler would have forced to create the copy constructor and assignment operator
 for this class object.
 Ofcourse, when there are more than one objects, such as
     HeapPtr<ClassName> _ptr;
     int abcd;
 then the compiler would synthesise the copy constructors just for the object int abcd and not for HeapPtr<>.
 Hence the moment, ClassName y;; ClassName x; y = x; is called, the compiler generates a compile error.

 When a compiler synthesises the Big Three, then it does it inline. Sometimes, however you would like it to be non
 inline, and hence it is good to write them explicitly.



 Copy constructor:
 Many a times, one would like to simply copy an already instantiated object.
 The copy constructor needs to be defined in the class. If the copy constructor definition does not exist, then the
 constructor cannot be copied as well. Using the copy constructor a class object can be initialised with an already
 available object. That means, the initialisation values are values of the object that is being copied.
 The copy constructor syntax can be
 Class obj2(obj1); Needs X-X reference Class(const Class& //mind, there is no parameter here//) { }
 Class obj2 = obj1; Needs Class& operator=(const Class& class);
 new CopyExample (&objectToBeCopied);
 new CopyExample( *this ) // when the copy is in the class itself.

 Default Constructor:
 This is of the form Class() { };
 If the base constructor is not called explicitly, then the compiler calls the default constructor written in the
 base class. In the below example, the default constructor has two variables, name and age and both are initialised
 to 0. However, if the base object would have been called by two parameters, then the initialisation would have been
 done according the parameters.

 If no base class constructor is specified, the default base class constructor will be used. In that case, if no
 default base class constructor can be found (or created by default), the compiler will display an error.

     Person(std::string name = "", int age = 0)
        : m_name(name), m_age(age )

 Person person1;   // Default constructor will be called
 Person person2 = Person();
 Person person3("Foo", 50); // the default constructor can have arguements, but the conditions is that the arguements
 should have a default value in the definition of the constructor. Please see below


 bool operator>(const Cents &c1, const Cents &c2)

 Consider the expression std::cout << Point. If the operator is <<, what are the operands? The left operand is the
 std::cout object of type std::ostream, and the right operand is the Point class object.

 If you are using a member function instead, the left operand automatically becomes the this object. If you are
 working on a unary operator, then no parameters needs to be passed.

 Although we can overload operator+(Cents, int) as a member function (as we did above), we can’t overload operator+
 (int, Cents) as a member function, because int isn’t a class we can add members to.

 Also operator<< cannot be overloaded with member function, because the left operand is std::ostream.

 Compiler implicitly converts an object prefix into a hidden leftmost parameter named *this


 friend ostream& operator<< (ostream &out, const Cents &cents)
 {
     out << cents.m_cents << " cents ";
     return out;
 }
 std::cout << average(array3, 4) << '\n';

 Friend function:
 Friend a functions is a breather for programmer because they do not have to define the getters and setters for the
 private variables. A friend function allows to access all the private variables of a class and hence gaining two
 important things - one does not have to define public functions for this class. This avoids any other class inheriting
 the class and using the public functions. The friend function is just like a normal function.  A friend function may
 be either a normal function, or a member function of another class.  To declare a friend function, simply use the
 friend keyword in front of the prototype of the function you wish to be a friend of the class.  It does not matter
 whether you declare the friend function in the private or public section of the class. Note that we have to pass the
 parent class object to the friend function. This is because friend function is not a member function. It does not have
 a *this pointer, nor does it have an parent class object to work with, unless given one. It is also possible to make
 an entire class a friend of another class. This gives all of the members of the friend class access to the private
 members of the other class. Here is an example:

    // Make the reset() function a friend of this class
    friend void reset(Accumulator &accumulator);

 This simply means, that reset() can be called from anyone. If we want to restrict reset to be called by a specific
 class, then NewClass::reset() needs to be forward declared in the ParentClass. Another way is to open the ParentClass
 completely by declaring the whole class as friend i.e

    friend class NewClass;

 in the prototype for the ParentClass. Friending is commonly used when defining overloading operators.
 It turns out that there are three different ways to overload operators: the member function way, the friend function
 way, and the normal function way. We’ll first show you the friend function way (because it’s more intuitive for most
 binary operators) and the normal function way. In a later lesson, we’ll cover the member function way (and discuss
 when to use each in more detail).
 impossible :     char c = 'Q';
    std::cout << &c;

 A ClassUsed defined as a friend in another ClassMain gets access to all the private variables of the ClassMain.
 Similarly a FunctionUsed() ( implicitly global ) defined as a friend in another ClassMain gets access to all the
 private variables of the ClassMain.
 But friending is not transitive. A ClassUsed2 defined as a friend in ClassUsed does not get access to the private
 variables of ClassMain. Similarly friendliness is not inherited.
 Friend functions ( not classes ) are however inheritable. i.e a derived class can use the friendliness of the base
 class.

 Friend functions differ from member functions in the way they are used. If an operator overloading such as << is
 declared as a member function, then the class object needs to be the left arguement. This is the only way, the class
 object can access the << operator for example classObject.operator<<. However we want the classObject on the right
 hand side, so that it can be streamed to the output stream cout. If the operator overloading is declared as a
 friend, then the classObject can be positioned to the right hand side. The best example is n.square or square(n). If
 you want to use square(n), then use friend in the function square. If you do not, then you will be forced to use the
 dot operator to access square.


 Const:
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
 Const Expression:
 Expressions that are evaluated at compile time are called const expressions
 constexpr int i = 0;
 constexpr char a = test1("Test", i); // also OK, compile time invocation of test1()
 constexpr char test3(unsigned i)
 {
    return t<i>::value;
 }

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
 function comes from a run time library. One point to note with respect to the virtual functions called in the
 constructors is that the constructors ignore the dynamic binding of virtual functions. The reason is simply as
 stated in the section Constructors is that the constructors give life to an object. One cannot go into the virtual
 world, when the object itself is not real. The same stands for destructors. By the time, the base class destructor
 is called, the derived object is already buried. Hence, it makes sense to call the native virtual function of the
 base class and not look for the derived version of the virtual function.

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

 There are two kinds of pointers: Raw pointer and Managed Pointer. The raw pointer is simply pointing to a location
 in the memory and can only be deleted by the delete keyword. It does not live in a container. Managed pointers live
 in a container and many actions can be done on the pointers. Most notably are destructing the pointer and the
 referent ( the object that the pointer is pointing to ) automatically.

 One should always use managed pointers like std::unique_ptr. Avoid std::shared_ptr although they are managed. The
 reason is that the shared_ptr allow to copy the pointers and hence can lead to dangling pointers after
 improper destruction.

 A very important thing is that * can only be used on basic data types. It is not possible to dereference a pointer
 of std::array<int,3> type with a *. However it is possible to dereference an int array[x][y] with *. For std::array
 or any other c++ class pointer, corresonding methods should be provided such as data() or data in case of std::array.

 Remote ownership:
 When the object that owns a pointer also owns the allocation pointed to by that pointer, the object is said to have
 remote ownership. That is the object owns the referent. And when the object has remote ownership, it is also
 responsible for deleting the referent.


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

 When a function returns a reference, the function call becomes an lvalue for the referent. This is normally used to
 allow operator expressions to be used as lvalues - the subscript operator, the dereference operator, and so on.
 functioncall(); = referent in the function i.e what is the reference referring to inside the function,

 Scope:

 Static data and functions inside a class are shared among all class objects. They can be accessed the way just another
 non  static data member or member function is accessed. The only difference is, that the static members retain their
 values and the  non static members have values depending on the class they are called.
 Another way to access static data member and functions is using the scope resolution operator. One can think static
 data and member functions as obdachlosen. They are not linked to any house and can be accessed by anyone. They also
 do not come under access control, hence they can be put anywhere ( private, public, protected ) and they are still
 accessible via the scope resolution operator or class objects. The static data is therefore default public. The
 static member and member functions when declared inside a class is just a kind of forward declaration. They have to
 be explicitly defined outside the class, because static member variables are not a part of the individual class
 objects. Mostly, the class is declared in the header file and the static members are defined in the cpp file. There
 is one excetion however, when the static values can also be defined within the class. This is when the static member
 is declared to be a const. This is called inline initialisation of static member variables.

 We said, that the static variables should be defined outside the class. This is not a good way to write a program,
 because in an object oriented programming, most of the data and initialisation is hidden. Constructors are meant to
 initialise and they are hidden. Similarly static member initialisation should also be hidden. The hiding of
 initialisation of static member data is possible, if the static object is initialised in a different class. So a
 sequence of nested classes can achieve the initialisation of static variables.

 Although static members can be accessed by the class object, this is discouraged. The perfereable way is to access
 the static member data via a scope resolution operator or the static member data can also be accessed by a static
 member function.
 One good example of the usage of static objects inside a class is when a count of number of objects of the class is
 required. Everytime a constructor is called, the static data increments and everytime a destructor is called, the
 static data is decremented. Ofcourse, one should make sure, that this static member data is not incremented or
 decremented elsewhere, which technically is not a problem at all.
 One word of caution is that the static objects needs to be initalised before it is used. Technically, it is possible
 that the static data is used before it is initialised. Proper care should be taken to avoid the situation.

 Static member functions cannot access non static data.
 Pure static class:
 A pure static claass is class whose all members are static. A word of note - pure virtual class is called abstract
 class. Pure virtual function is when a function is initialised to 0.


 Class scope static objects
 File scope static objects
 File scope global objects

 TYPEDEFS and #DEFINES

    typedef int* int_p1;
    int_p1 a, b, c;  // a, b, and c are all int pointers.

    #define int_p2 int*

    int_p2 a, b, c;  // only the first is a pointer!

    typedef int a10[10];
    a10 a, b, c; // create three 10-int arrays
    int a[10], b[10], c[10];
    typedef int (*func_p) (int);
    func_p fp; // fp is a pointer to a function that takes an int and returns an int
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

 typedef int &ref_to_int; ref_to_int const r = i;

 Typecasts:
 static_cast : for anything other than base and derived pointer
 const_cast : convert constness
 dynamic_cast : for base and derived pointer
 reinterpret_cast:

 Exception:
 We assert a statement. i.e assert(true) is OK. But assert(false) will trigger an exception. If the condition inside
 the assert is correct, then no exception will be thrown.
 assert(found && "Car could not be found in database"); // Add a string
 static_assert(sizeof(long) == 8, "long must be 8 bytes"); This is triggered only at compile time.

 std::vector:
 A vector is a dynamic array with automatically handled storage. The elements in a vector can be accessed just as
 efficiently as those in an array with the advantage being that vectors can dynamically change in size

 std::array:
 std::array is a container that encapsulates fixed size arrays. Unlike A C-style array, it doesnt decay to T*
 automatically. As an aggregrate type, it can be initialized with aggregrate-initialisation given at most N
 initializers that are convertible to T.

 Compiler Options:
 -std=c++11
 -std=gnu++11
 -fexception

 Aggregrates:
 Aggregrates is an array or a class ( struct , union inclusive ) with
 - no explicitly user declared constructors ( default, non-default and copy constructors ). Aggregrates can have user
 defined  assignment operator though.
 - only public data members.
 - no base class
 - no virtual functions.
 - An array is an aggregrate even if it is an array of non aggregrate class type.
 A special thing about aggregrates are that they can be initialised with curly braces { }. Non aggregrates cannot be
 initialised with curly brackets.
 Initialisation of aggregrate array is simple. Simply use the curly brackets to initialise them.
 Initialisation of aggregrate class is complex. The elements in curly brackets are assigned one to one in the order
 of the public data members that appear in the class declaration. Memberwise initialisaton with braces implies that
 the class is nothing more than the sum of its members.
 In C++11, one can have a user defined default constructor. But it just points to the default constructor.
 Aggregate() = default; // asks the compiler to generate the default implementation
 Also, any kind of inline initialization of the data members in the class declarations, such as int x = 5; violates
 the principles of an aggregrate. Such kind of inline initialization is similiar to a user defind default constructor
 when the class is instantiated.
 Aggregrate initialisation can be folded in during compile time. For example std::array is a compile time type and
 hence needs to be work with aggregrate initialisation.

 PODs ( Plain Old Data :
 If a class is not an aggregreate, then it is definetly not a POD. A POD is a special type of aggregrate with few
 more constraints. As we saw, that an aggregrate can consist of assignment operator.  A POD on the other hand
 - has no user defined copy-assignment operator
 - none of its non static data members is a non POD class. As soon as the any of static data members is a non POD class,
 then the POD class is not a POD anymore.
 - none of the non stattic data members are references.
 - none of the non static data members are array of non POD class.
 - should not have a user defined destructor. The aggregrates allow a user defined destructor.
 In a nutshell it means, that the PODs cannot consist of any hint of non PODs. This is different from aggregrates,
 where the aggregrate class can be an array of non aggregrates.
 A POD is closest to C struct, with additional member functions. PODs are optimum for potability in dynamic libraries
 . They are often used as portable dynamic library. The lifetime of a POD ends with the release or reuse of the
 storage, unlike Aggregrates, where the execution of the destructor ends the lifetime of the object. Memcpy from the
 POD type object to a array of char and back, will not alter the object. This cannot be gauranteed for the non POD
 type. A struct memcpy to an array and back will reveal the original struct again for example. Additionally member
 functions in a POD type extends the struct kind of objects.

 Randomize:
 std::generate, std::generate_n, std::nrand()
 Fischer-Yates Shuffle, std::generate_random()

 std::generate(array, array + sizeof(array)/sizeof(int), ::rand);
 std::transform(array, array+SIZE, array, std::bind2nd(std::modulus<int>(), 255));

 srand( unsigned( time(NULL) ) );
 random_shuffle(array, array+5);

 std::string uniqueName() {
    auto randchar = []() -> char
    {
        const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(4,0);
    std::generate_n( str.begin(), 4, randchar );
    return str;
 }





*/



