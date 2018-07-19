//
// Created by veikas on 26.06.18.
//

#ifndef MAIN_CPPOBJECTS_H
#define MAIN_CPPOBJECTS_H

#include "BasicObjects.h"

class CppObjects: public BasicObjects {


public:

    CppObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): BasicObjects(current_sensor_group_index, generatepath) {}

    CppObjects() {}

    void process(std::unique_ptr<Noise> &noise);



};


#endif //MAIN_CPPOBJECTS_H