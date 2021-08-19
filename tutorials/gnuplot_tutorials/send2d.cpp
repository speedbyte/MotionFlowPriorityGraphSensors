

#define USE_CXX (__cplusplus >= 201103)

#include <vector>
#include <complex>
#include <cmath>

#include <boost/tuple/tuple.hpp>
#include <boost/array.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/irange.hpp>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>

#include <gnuplot-iostream/gnuplot-iostream.h>

#ifndef M_PI
#	define M_PI 3.14159265358979323846
#endif

#define PI boost::math::constants::pi<long double>()


void gnuplot_from_file() {

    const char *filename_gnuplot = "liveplot.gnu";
    const char *filename_data = "plot.dat";
    boost::filesystem::remove(filename_gnuplot);
    boost::filesystem::remove(filename_data);

    // save file in a filename and call gnuplot
    Gnuplot gp(std::string("tee ") + std::string(filename_gnuplot) + std::string(" | gnuplot -persist"));

    for ( unsigned i = 0; i < 10; i++ ) {
        FILE *fp = fopen(filename_data, "a");
        fprintf(fp, "%d\t%d\t%d\n", i, i*i,i);
        fclose(fp);
        gp << "set xrange [0:20]\n";
        gp << "set yrange [0:400]\n";
        gp << "set zrange [0:10]\n";
        gp << "splot \"" << filename_data << "\" with points\n";
        gp << "pause 1\n";
        gp << "reread\n";
    }
}


void gnuplot_from_commandline() {

    // Example copied from http://gnuplot.sourceforge.net/demo/surface1.9.gnu
    Gnuplot gp("tee create_file.gp | gnuplot -persist");
    gp << "set samples 21, 21\n";
    gp << "set isosamples 11, 11\n";
    gp << "set style data lines\n";
    gp << "set title \"3D gnuplot demo\"\n";
    gp << "set xlabel \"X axis\"\n";
    gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
    gp << "set xrange [ -10.0000 : 10.0000 ] noreverse nowriteback\n";
    gp << "set ylabel \"Y axis\"\n";
    gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
    gp << "set yrange [ -10.0000 : 10.0000 ] noreverse nowriteback\n";
    gp << "set zlabel \"Z axis\"\n";
    gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
    gp << "DEBUG_TERM_HTIC = 119\n";
    gp << "DEBUG_TERM_VTIC = 119\n";
    gp << "splot x*y with points\n";
}

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
    Gnuplot gp;

    gp << "set zrange [0:1]\n";
    gp << "set hidden3d nooffset\n";

    // I use temporary files rather than stdin because the syntax ends up being easier when
    // plotting several datasets.  With the stdin method you have to give the full plot
    // command, then all the data.  But I would rather give the portion of the plot command for
    // the first dataset, then give the data, then the command for the second dataset, then the
    // data, etc.

    gp << "splot ";

    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double> > >  pts;
    //std::vector<std::vector<std::vector<double> > > pts(3);

    std::vector<double> x_pts, y_pts, z_pts;


    for (int v = 0; v < 25; v++) {
        double z = 0;
        double x = std::cos(2.0 * M_PI * v / (24));
        double y = std::sin(2.0 * M_PI * v / (24));

        x_pts.push_back(x);
        y_pts.push_back(y);
        z_pts.push_back(z);
    }

    pts.push_back(std::make_tuple(x_pts, y_pts, z_pts));
    gp << gp.binFile2d(pts, "record") << "with lines title 'object1'";
    gp << std::endl;

}

void send_lemniscate() {

    Gnuplot gp;

    gp << "set zrange [1:12]\n";
    gp << "set hidden3d nooffset\n";

    // I use temporary files rather than stdin because the syntax ends up being easier when
    // plotting several datasets.  With the stdin method you have to give the full plot
    // command, then all the data.  But I would rather give the portion of the plot command for
    // the first dataset, then give the data, then the command for the second dataset, then the
    // data, etc.



    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double> > > pts;
    std::vector<signed> theta;
    std::vector<double> x_pts, y_pts, z_pts, b_pts;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 360);

    const int runs = 5;
    int numRuns = 0;

    for (int n = 0; n < 100; n++) {
        //Use dis to transform the random unsigned int generated by gen into a double in [1, 2)
        theta.push_back(dis(gen)); //dis(gen)
        std::cout << theta[n] << ' '; //Each call to dis(gen) generates a new random double
    }

    while (numRuns++ < runs) {

        std::cout << '\n';

        std::string cx, cy, cz;
        // Lemniscate Noise Model
        for (int i = 0; i < 100; i++) {
            // Noise
            double x, y, z;
            x = 1 * cos(theta[i] * PI / 180.0) / (1.0 + std::pow(sin(theta[i] * PI / 180.0), 2));
            y = 1 * (cos(theta[i] * PI / 180.0) * sin(theta[i] * PI / 180.0)) / (0.2 + std::pow(sin(theta[i] * PI / 180.0), 2));

            x_pts.push_back(x);
            y_pts.push_back(y);
            z_pts.push_back(4); //(double) numRuns * 2

            if (i == numRuns) {
                cx = std::to_string(x);
                cy = std::to_string(y);
                cz = std::to_string(4);
            }

            // Real
        }

        std::string filename = std::string("record") + std::to_string(numRuns) + std::string(".dat");

        gp << "set label \"PT\" at " + cx + "," + cy + "," + cz + "; ";
        pts.push_back(std::make_tuple(x_pts, y_pts, z_pts));
        // lesbare datei
        //gp << "splot " << gp.file2d(pts, filename) << "with points title 'object1'";
        // binaer datei
        gp << "splot " << gp.binFile2d(pts, "record", filename) << "with points title 'object1'";
        gp << std::endl;

        pts.clear();
        x_pts.clear();
        y_pts.clear();
        z_pts.clear();

        std::cout << "Sleep" << std::endl;
        usleep(500 * 1000);

    }

}


int main() {

    //send3d();
    //send3d_record();
    //send2d_colmajor();
    send_lemniscate();
    //gnuplot_from_file();
    //gnuplot_from_commandline();

    return 0;
}


