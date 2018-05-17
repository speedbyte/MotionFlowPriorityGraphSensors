//
// Created by veikas on 07.02.18.
//

#ifndef MAIN_SIMULATEDOBJECTS_H
#define MAIN_SIMULATEDOBJECTS_H


#include <opencv2/core/types.hpp>
#include "Objects.h"

class SimulatedObjects : public Objects {
public:

    static unsigned SimulatedobjectCurrentCount; // assingn object id

private:


public:

    SimulatedObjects(std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) :
            Objects(objectName, width, height, extrapolated_visibility ) {

        m_objectId = SimulatedobjectCurrentCount ;
        SimulatedobjectCurrentCount += 1;
    }

};


#endif //MAIN_SIMULATEDOBJECTS_H
