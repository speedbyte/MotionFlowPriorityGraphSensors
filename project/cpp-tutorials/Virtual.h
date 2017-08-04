//
// Created by veikas on 23.07.17.
//

#ifndef CPP_TUTORIALS_VIRTUAL_H
#define CPP_TUTORIALS_VIRTUAL_H

/** brief
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
 */

namespace cpp_tutorials {
    class Animal {
    public:
        void /*non-virtual*/ move(void) {
            std::cout << "This animal moves in some way" << std::endl;
        }
        virtual void eat(void) = 0;
    };

    // The class "Animal" may possess a definition for eat() if desired.
    class Llama : public Animal {
    public:
        // The non virtual function move() is inherited but not overridden
        void eat(void) override {
            std::cout << "Llamas eat grass!" << std::endl;
        }
    };

    class Super
    {
        public:
            void iAm(int a) { std::cout << "I'm the super class!\n"; }
            virtual void iAmHello() { std::cout << "I'm not !\n"; }
    };

    class Sub : public Super
    {
        public:
            void iAmHello() override  { std::cout << "I'm the subclass!\n"; }
    };

    class Cents {
    private:
        int m_cents;

    public:
        Cents(int cents) : m_cents ( cents ){};

        friend int operator + (const Cents &c1, const Cents &c2) {
            return (c1.m_cents + c2.m_cents);
        }
        int getCents(void) {
            return m_cents;
        }
        friend std::ostream& operator<< (std::ostream &out, const Cents &cents) {
            out << cents.m_cents << " cents ";
            return out;
        }
        void operator+=(Cents cents) {
            m_cents += cents.m_cents;
        }

        void operator/=(int value) {
            m_cents /= value;
        }
    };
}


#endif //CPP_TUTORIALS_VIRTUAL_H
