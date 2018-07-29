


#include <random>
#include <iostream>
#include <map>
#include <iomanip>
#include <algorithm>
#include <cfloat>
#include <functional>

int main (int argc, char *argv[]) {

    std::cout << FLT_MAX << " " << INT32_MIN << " " << INT32_MAX << " " << std::numeric_limits<float>::infinity() << std::endl;

    std::vector<unsigned> unsigned_vector(100);

    for ( auto a:unsigned_vector) {
        std::cout << rand()%255 << std::endl;

    }

    float rand_range = rand()%100;

    time_t rawtime;
    time(&rawtime);
    std::cout << asctime(localtime(&rawtime));

    // Combination of default_random_engine and uniform_distribution
    // An engine is a function object that generates a uniformly distributed sequence of integer values.
    std::default_random_engine set_of_numbers(6);
    std::cout << set_of_numbers;
    // A distribution is a function object that generates a sequence of values according to a mathematical formula given a sequence of values from an engine as inputs.
    std::uniform_int_distribution<int> uniform_dist(1, 6);
    int mean = std::uniform_int_distribution<int>{1,6}(set_of_numbers); //uniform_dist(set_of_numbers);

    auto gen = std::bind(std::uniform_int_distribution<int>{1,6}, set_of_numbers);
    mean = gen();
    auto gen2 = std::bind(std::uniform_int_distribution<int>{1,6}, set_of_numbers);
    mean = gen2();

    std::cout << "Randomly-chosen mean: " << mean << '\n';


    // Combination of random_device_engine and uniform_distribution
    // Seed with a real random value, if available
    std::random_device r;
    std::cout << r() << std::endl;

    // Generate a normal distribution around that mean
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 e2(seed2); // distribution with random_device_engine


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

    // print out content:
    std::cout << "myvector contains:";
    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
        std::cout << ' ' << *it;

    // using myrandom:
    //std::random_shuffle ( myvector.begin(), myvector.end(), (std::rand()));

    // print out content:
    std::cout << "myvector contains:";
    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
        std::cout << ' ' << *it;

    //randn(m1,mean,stddev);

}