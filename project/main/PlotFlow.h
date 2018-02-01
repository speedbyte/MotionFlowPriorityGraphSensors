//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_PLOTFLOW_H
#define MAIN_PLOTFLOW_H

#include <iostream>
#include "Dataset.h"
#include "FlowImageExtended.h"

class PlotFlow : public FlowImageExtended {

public:
    static void plot(const std::string &resultsordner);

};


#endif //MAIN_PLOTFLOW_H
