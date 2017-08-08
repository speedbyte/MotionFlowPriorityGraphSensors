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

 #168 Constructors and Destructors







 */

#endif //CPP_TUTORIALS_INTRODUCTION_H
