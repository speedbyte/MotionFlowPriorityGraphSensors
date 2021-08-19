
#include <chrono>
#include <iostream>

/**
struct tm {
    int tm_sec;  // second of minute [0:61]; 60 and 61 represent leap seconds
    int tm_min;  // minute of hour [0,59]
    int tm_hour;     // hour of day [0,23]
    int tm_mday;    // day of month [1,31]
    int tm_mon;     // month of year [0,11]; 0 means January (note: not [1:12])
    int tm_year;     // year since 1900; 0 means year 1900, and 102 means 2002
    int tm_wday; // days since Sunday [0,6]; 0 means Sunday
    int tm_yday;   // days since January 1 [0,365]; 0 means January 1
    int tm_isdst;    // hours of Daylight Savings Time
};
*/
int main () {
    clock_t clock();      // number of clock ticks since the start of the program

    time_t time(time_t* pt);                             // current calendar time
    double difftime(time_t t2, time_t t1);     // t2–t1 in seconds

    tm* localtime(const time_t* pt);   // local time for the *pt
    tm* gmtime(const time_t* pt);     // Greenwich Mean Time (GMT) tm for *pt, or 0

    time_t mktime(tm* ptm);               // time_t for *ptm, or time_t(–1)

    char* asctime(const tm* ptm);     // C-style string representation for *ptm
    char* ctime(const time_t* t) { return std::asctime(localtime(t)); }

    int n = 100000;
    auto t1 = std::chrono::system_clock::now();
    for ( int I = 0; i < n; i++ );
    auto t2 = std::chrono::system_clock::now();

    std::cout << duration_cast<std::chrono::milliseconds>(t2-t1).count();
}
