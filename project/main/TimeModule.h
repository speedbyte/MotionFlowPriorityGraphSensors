//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_TIMEMODULE_H
#define MAIN_TIMEMODULE_H


#include <iosfwd>

class TimeModule {

private:
    double time;

    friend std::ostream& operator<< ( std::ostream &out, TimeModule timeModule ) {
        out << timeModule.time;
    }
};


#endif //MAIN_TIMEMODULE_H
