//
// Created by veikas on 23.07.17.
//

#include "Virtual.h"

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
        virtual void iAm(int a) { std::cout << "I'm the super class!\n"; }
        void iAmHello() { std::cout << "I'm not !\n"; }
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
        friend std::ostream& operator<< (std::ostream &out, const Cents &cents)
        {
            out << cents.m_cents << " cents ";
            return out;
        }
        void operator+=(Cents cents)
        {
            m_cents += cents.m_cents;
        }
        void operator/=(int value)
        {
            m_cents /= value;
        }
    };
}
