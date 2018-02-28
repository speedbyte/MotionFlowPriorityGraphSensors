//
// Created by veikas on 23.01.18.
//

#include <vector>
#include <memory>
#include <algorithm>
#include "ClassTutorials.h"

void someFunction(std::vector<std::unique_ptr<Base> > & vec) {

    std::vector<std::unique_ptr<Base>>::iterator it = vec.begin();
    std::vector<std::unique_ptr<Base>>::iterator it_next = it+1;

    std::vector<std::pair<std::unique_ptr<Base>, std::unique_ptr<Base>> > pairWise;
    //pairWisevectorUnique.push_back(std::make_pair( (*it).get(), (*it_next).get() ));
    //pairWisevectorUnique.push_back(std::make_pair( (*it).get(), (*it_next).get() ));

    //pairWise.push_back(std::make_pair(std::make_unique<Base>(*(*it).get()), std::make_unique<Base>(*(*it_next).get())));
    //pairWise.push_back(std::make_pair(std::make_unique<Base>(*(*it).get()), std::make_unique<Base>(*(*it_next).get())));
    //pairWise.emplace_back(std::make_unique<Base>(*(*it).get()), std::make_unique<Base>(*(*it_next).get()));

    std::cout << pairWise.at(0).first.get()->getA() << std::endl;
    //std::cout << "enddd\n";
    std::cout << pairWise.at(1).second.get()->getA() << std::endl;


}

void someFunction2(std::vector<Base*> &vec) {

    std::vector<Base*>::iterator it = vec.begin();
    std::vector<Base*>::iterator it_next = it+1;

    std::vector<std::pair<Base*, Base*> > pairWise;
    //pairWisevectorUnique.push_back(std::make_pair( (*it).get(), (*it_next).get() ));
    //pairWisevectorUnique.push_back(std::make_pair( (*it).get(), (*it_next).get() ));

    pairWise.push_back(std::make_pair( ((*it)), ((*it_next))) );
    pairWise.push_back(std::make_pair( ((*it)), ((*it_next))) );

    std::cout << pairWise.at(0).first->getA() << std::endl;
    //std::cout << "enddd\n";
    std::cout << pairWise.at(1).second->getA() << std::endl;


}


int main(int argc, char *argv[]) {


    Rectangle rect;
    GroundTruth obj1 = GroundTruth(rect);
    Noise noise;

    std::vector<GroundTruth> list_objects;
    list_objects.push_back(obj1);

    //Object obj1 = ObjectList(list_rect);

    std::cout << obj1.getShape() << std::endl;
    std::cout << list_objects.at(0).getShape() << std::endl;

    /* ------- */

    Base a;
    Derived b;
    Derived2 c;

    Base &d = b;

    a.printA();
    std::cout << std::endl;
    b.printB();
    std::cout << std::endl;
    c.printC();

    b.setA(10);
    b.printA();

    std::vector<Derived> derivedObjects(2);
    derivedObjects.at(0).setA(150);
    derivedObjects.at(1).setA(300);
    std::vector<Base*> vecPtr = {&derivedObjects.at(0), &derivedObjects.at(1)};

    std::vector<Derived> derivedObjectsBase;
    derivedObjectsBase.push_back(derivedObjects.at(0));
    //derivedObjectsBase = derivedObjects;


    /*
    std::transform(derivedObjects.begin(), derivedObjects.end(), std::back_inserter(vecPtr),
                   [](auto p){ return std::make_unique<Base>(p); });
*/
    someFunction2(vecPtr);


    //std::cout << b.getA();

    /* -------------- */

    Rectangle rect1;
    std::cout << rect1.getXyz() << std::endl;



}