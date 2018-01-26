//
// Created by veikas on 23.01.18.
//

#include "ClassTutorials.h"

int main(int argc, char *argv[]) {

    Base a;
    Derived b;
    Derived2 c;

    a.printA();
    std::cout << std::endl;
    b.printB();
    std::cout << std::endl;
    c.printC();

    //std::cout << b.getA();

    Rectangle rect;
    Object obj1 = Object(rect);

    std::cout << obj1.getShape() << std::endl;



}