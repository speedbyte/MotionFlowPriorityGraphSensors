

#define USE_CXX (__cplusplus >= 201103)

#include <vector>
#include <complex>
#include <cmath>

#include <boost/tuple/tuple.hpp>
#include <boost/array.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/irange.hpp>
#include <boost/bind.hpp>


#include <gnuplot/gnuplot-iostream.h>

#ifndef M_PI
#	define M_PI 3.14159265358979323846
#endif


void send3d() {
    Gnuplot gp(stdout);

    gp << "set xrange [0:10]\nset yrange [0:10]\nset zrange [0:10]\n";
    //gp << "set hidden3d nooffset\n";

    std::vector<boost::tuple<std::vector<double>,std::vector<double>,std::vector<double> > > pts;
    std::vector<double> x_pts(200);
    std::vector<double> y_pts(200);
    std::vector<double> z_pts(200);

    srand(time(NULL));
    for(int v=0; v<200; v++) {
        x_pts[v] = rand()%10;
        y_pts[v] = rand()%10;
        z_pts[v] = rand()%10;
    }
    pts.push_back(boost::make_tuple(x_pts, y_pts, z_pts));
    gp << "splot '-' with points title 'vec of boost::tuple of vec'\n";
    gp.send2d(pts);
    gp << std::endl;

}

void send3d_record() {

        Gnuplot gp;

        gp << "set xrange [0:10]\nset yrange [0:10]\nset zrange [0:10]\n";
        //gp << "set hidden3d nooffset\n";

        std::vector<boost::tuple<std::vector<double>,std::vector<double>,std::vector<double> > > pts;
        std::vector<double> x_pts(200);
        std::vector<double> y_pts(200);
        std::vector<double> z_pts(200);
        srand(time(NULL));
        for(int v=0; v<200; v++) {
            x_pts[v] = rand()%10;
            y_pts[v] = rand()%10;
            z_pts[v] = rand()%10;
        }
        pts.push_back(boost::make_tuple(x_pts, y_pts, z_pts));
        gp << "splot" << gp.binFile2d(pts, "record") << " with lines title 'vec of boost::tuple of vec'\n";
        gp << std::endl;

};

void send2d_colmajor() {
    //Gnuplot gp;
    // for debugging, prints to console
    Gnuplot gp(stdout);

    gp << "set zrange [0:1]\n";
    gp << "set hidden3d nooffset\n";

    // I use temporary files rather than stdin because the syntax ends up being easier when
    // plotting several datasets.  With the stdin method you have to give the full plot
    // command, then all the data.  But I would rather give the portion of the plot command for
    // the first dataset, then give the data, then the command for the second dataset, then the
    // data, etc.

    gp << "splot ";

    std::vector<std::vector<std::vector<double> > > pts(3);

    for (int i = 0; i < 3; i++) pts[i].resize(5);

    for (int u = 0; u < 5; u++) {
        for (int i = 0; i < 3; i++) pts[i][u].resize(25);
    }

    for (int u = 0; u < 5; u++) {
        for (int v = 0; v < 25; v++) {
            double z = 0;
            double x = std::cos(2.0 * M_PI * v / (24));
            double y = std::sin(2.0 * M_PI * v / (24));

            pts[0][u][v] = x;
            pts[1][u][v] = y;
            pts[2][u][v] = z;
        }
    }
    gp << gp.binFile2d_colmajor(pts, "record") << "with lines title 'object1'";
    gp << std::endl;

}

int main() {

    send3d();
    send3d_record();
    //send2d_colmajor();

    return 0;
}


