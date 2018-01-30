//
// Created by veikas on 23.01.18.
//

#ifndef CPP_TUTORIALS_CLASSTUTORIALS_H
#define CPP_TUTORIALS_CLASSTUTORIALS_H

#include <iostream>

class Shape;

class Noise {

public:
    void changeValue(Shape &shape) {
        shape.set(50);
    }

};

class Shape {

protected:
    int shape = 10;
public:

    int get() {
        return shape;
    }
    virtual void process() {
        shape = 30;
    }

    virtual void set(int x) {

    }
};

class Rectangle : public Shape {

    int xyz = 100;

public:
    void process() override {
        shape = 40;
    }

    void changexyz(Noise noise) {
        noise.changeValue(*this);
    }

    void set(int x) override {
        xyz = x;
    }

    int getXyz() {
        return xyz;
    }

};


class GroundTruth : Shape {

    Shape &m_sh;
public:
    GroundTruth(Shape &sh) : m_sh ( sh) {
        //m_sh = sh;
        m_sh.process();
    }

    int getShape() {
        return m_sh.get();
    }

};

/* ------------------- */

class Base {
protected:
    int a;
public:
    void printInherit() {
        std::cout << "in base inherit\n";
        printDerived();
    }

    // Virtual functions must be implemented or set to 0
    virtual void printDerived()  {
        a  = 10;
        std::cout << "virtual base to be printed " << a << std::endl;
    }

    void printA() {
        std::cout << "in Base printA\n";
        printDerived();
    }

    int getA() {
        return a;
    }
};

class Derived: public Base {
    virtual void printDerived() override {
        a = 20;
        std::cout << "virtual derived to be printed " << a << std::endl;
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
    void printDerived() override {
        std::cout << "virtual derived2 to be printed\n";
    }

public:
    void printC() {
        std::cout << "in Derived2\n";
        printInherit2();
    }
};


#endif //CPP_TUTORIALS_CLASSTUTORIALS_H
