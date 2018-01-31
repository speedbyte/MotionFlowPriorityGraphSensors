//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_PLOTFLOW_H
#define MAIN_PLOTFLOW_H

#include <iostream>
#include "Dataset.h"
#include <kitti/io_flow.h>

class PlotFlow : public FlowImage {

public:
    static void plot(const Dataset &dataset, const std::string &resultsordner);

};


#endif //MAIN_PLOTFLOW_H
