


#include <random>
#include <iostream>
#include <map>
#include <iomanip>
#include <algorithm>
#include <cfloat>

int main (int argc, char *argv[]) {

    std::cout << FLT_MAX << " " << INT32_MIN << " " << INT32_MAX;

    time_t rawtime; time(&rawtime);
    std::cout << asctime(localtime(&rawtime));

    // Seed with a real random value, if available
    std::random_device r;

    // Choose a random mean between 1 and 6
    std::default_random_engine e1(r());
    std::uniform_int_distribution<int> uniform_dist(1, 6);
    int mean = uniform_dist(e1);
    std::cout << "Randomly-chosen mean: " << mean << '\n';

    // Generate a normal distribution around that mean
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 e2(seed2);
    std::normal_distribution<> normal_dist(mean, 2);

    std::map<int, int> hist;
    for (int n = 0; n < 10000; ++n) {
        ++hist[std::round(normal_dist(e2))];
    }
    std::cout << "Normal distribution around " << mean << ":\n";
    for (auto p : hist) {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
    std::srand ( unsigned ( std::time(0) ) );
    std::vector<int> myvector;

    // set some values:
    for (int i=1; i<10; ++i) myvector.push_back(i); // 1 2 3 4 5 6 7 8 9

    // using built-in random generator:
    std::random_shuffle ( myvector.begin(), myvector.end() );

    // using myrandom:
    //std::random_shuffle ( myvector.begin(), myvector.end(), (std::rand()));

    // print out content:
    std::cout << "myvector contains:";
    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
        std::cout << ' ' << *it;

    //randn(m1,mean,stddev);

}