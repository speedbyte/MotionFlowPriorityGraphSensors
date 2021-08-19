

#include <gnuplot-iostream/gnuplot-iostream.h> // this depends on the boost library. Also adds
#include <iostream>
#include <array>
#include <map>


float factorial ( int n ) {
    return ((n == 1 || n == 0) ? 1 : factorial(n - 1) * n);
}

void bionomialDistribution() {

    Gnuplot gp;

    std::vector<std::pair<int, float>> xy_pts;

    int X=15;
    std::vector<int> random_variable_set(X);// Difference in the number appearing on the two dices.
    for ( unsigned x = 0; x < random_variable_set.size(); x++) {
        random_variable_set[x] = x;
    }
    std::vector<float> random_bionomial_distribution(X,0);
    float p = 0.5,q = 0.5;
    int n = 15;
    float probability_total = 0;
    for ( unsigned i=0; i<random_variable_set.size(); i++) {
        random_bionomial_distribution[i] = (float)
                                                       (std::pow(p,random_variable_set.at(i)) *
                                                        std::pow(q,(n-random_variable_set.at(i))) *
                                                        factorial(n) ) /
                                                       (factorial(random_variable_set.at(i)) *
                                                                          (factorial(n-random_variable_set.at(i))));
        xy_pts.push_back(std::make_pair(i,random_bionomial_distribution.at(i)));
        probability_total += random_bionomial_distribution.at(i);
        std::cout << random_bionomial_distribution.at(i) << std::endl;
    }

    std::cout << "probability total " << probability_total << std::endl;

    gp << "set xrange [0:"<< std::to_string(X) <<"]\n";
    gp << "set yrange [0:1]\n";
    gp << "set style function dots\n";
    gp << "set title 'discrete bionomial distribution'\n";
    gp << "set xlabel 'Number of defective items'\n";
    gp << "set ylabel 'probability'\n";
    gp << "plot '-' with impulses\n";
    gp.send1d(xy_pts);


}

void discreteDistribution() {

    Gnuplot gp;
    std::vector<std::pair<int, float>> xy_pts;

    std::vector<int> sample1 = {1,2,3,4,5,6};
    std::vector<int> sample2 = {1,2,3,4,5,6};
    std::vector<int> random_variable_set = {0,1,2,3,4,5}; // Difference in the number appearing on the two dices.
    std::vector<int> random_variable_outcome = {6,10,8,6,4,2}; // Number of ways the difference can appear.
    std::vector<float> random_variable_distribtuion;
    float probability_total = 0;
    for ( unsigned i=0; i<random_variable_set.size(); i++) {
        random_variable_distribtuion.push_back((float)random_variable_outcome.at(i)/36.0);
        xy_pts.push_back(std::make_pair(i,random_variable_distribtuion.at(i)));
        probability_total += random_variable_distribtuion.at(i);
    }

    std::cout << "probability total " << probability_total << std::endl;

    gp << "set xrange [0:6]\n";
    gp << "set yrange [0:1]\n";
    gp << "set xlabel '|dice1-dice2|'\n";
    gp << "set ylabel 'probability'\n";
    gp << "set title 'discrete distribution |dice1-dice2|'\n";
    gp << "plot '-' with impulses \n";
    gp.send1d(xy_pts);

}

void uniformDistribution() {
    // Sample space for the actual water demand and the electricity demand. The sample space has infinitely many
    // points. Indeed the sample space is, uncountable.
    Gnuplot gp;
    std::vector<float> water_demand(200);
    std::vector<float> electricity_demand(150);
    std::vector<std::pair<unsigned, float>> xy_pts;

    srand( unsigned( time(NULL) ) );
    std::vector<std::string> events;
    std::map<int,std::string> map_events;
    events.push_back("high_demand");
    events.push_back("water_50_175");

    for (unsigned i=0; i<events.size() ; i++) {
        map_events[0] = events[0];
    }
    int totalarea = (150-1)*(200-4);
    std::vector<float> probability_water_demand(201);
    // Probability distribution of water demands.
    for ( unsigned len = 4; len <= 200; len++) {
        probability_water_demand.at(len) = (float)((149) * len) / totalarea;
        xy_pts.push_back(std::make_pair(len, probability_water_demand.at(len)));
    }

    gp << "set xrange [0:200]\n";
    gp << "set yrange [0:1]\n";
    gp << "set xlabel 'water demand'\n";
    gp << "set ylabel 'probability'\n";
    gp << "set title 'Cumulative Uniform Distribution'\n";
    gp << "plot '-' with points'\n";
    gp.send1d(xy_pts);

}
int main ( int argc, char *argv[]) {

    uniformDistribution();
    discreteDistribution();
    bionomialDistribution();

}
