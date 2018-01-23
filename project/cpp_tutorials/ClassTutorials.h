//
// Created by veikas on 23.01.18.
//

#ifndef CPP_TUTORIALS_CLASSTUTORIALS_H
#define CPP_TUTORIALS_CLASSTUTORIALS_H

#include <iostream>

class Base {
public:
    void printInherit() {
        std::cout << "in base inherit\n";
        printDerived();
    }

    virtual void printDerived() {
        std::cout << "virtual base to be printed\n";
    }

    void printA() {
        std::cout << "in Base printA\n";
        printDerived();
    }

};

class Derived: Base {
    virtual void printDerived() {
        std::cout << "virtual derived to be printed\n";
    }

public:
    void printB() {
        std::cout << "in Derived\n";
        printInherit();
    }

    void printInherit2() {
        std::cout << "in derived inherit\n";
        printInherit();
    }

};

class Derived2: Derived {
    void printDerived() {
        std::cout << "virtual derived2 to be printed\n";
    }

public:
    void printC() {
        std::cout << "in Derived2\n";
        printInherit2();
    }

};


#endif //CPP_TUTORIALS_CLASSTUTORIALS_H
