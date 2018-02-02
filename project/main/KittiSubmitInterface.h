//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_KITTISUBMITINTERFACE_H
#define MAIN_KITTISUBMITINTERFACE_H


#include "FlowImageExtended.h"

class KittiSubmitInterface {

public:
    int sendData(int argc, char *argv[]);

    std::vector<float> flowErrorsOutlier (FlowImageExtended &F_gt,FlowImageExtended &F_orig,FlowImageExtended &F_ipol);

    vector<float> flowErrorsAverage (FlowImageExtended &F_gt,FlowImageExtended &F_orig,FlowImageExtended &F_ipol);

    void plotVectorField (FlowImageExtended &F,std::string dir,char* prefix);

    bool eval (std::string result_sha, Mail* mail);

};


#endif //MAIN_KITTISUBMITINTERFACE_H
