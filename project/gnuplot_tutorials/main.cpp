

void plot() {
    Gnuplot gp;
    std::vector<std::pair<double, double> > xy_pts_A;
    for(double x=-2; x<2; x+=0.01) {
        double y = x*x*x;
        xy_pts_A.push_back(std::make_pair(x, y));
    }

    std::vector<std::pair<double, double> > xy_pts_B;
    for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
        double theta = alpha*2.0*3.14159;
        xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
    }

    gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
    gp << "plot '-' with lines title 'real', '-' with points title 'noise'\n";
    gp.send1d(xy_pts_A);
    gp.send1d(xy_pts_B);

    //pause_if_needed();
}
