//
// Created by veikas on 23.01.18.
//

#include <vector>
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
    GroundTruth obj1 = GroundTruth(rect);

    std::vector<GroundTruth> list_objects;
    list_objects.push_back(obj1);

    //Object obj1 = ObjectList(list_rect);


    std::cout << obj1.getShape() << std::endl;
    std::cout << list_objects.at(0).getShape() << std::endl;



}