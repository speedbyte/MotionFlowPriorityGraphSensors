
/**
    gp(std::fopen("plot.gnu"));
    gp("tee plot.gnu | gnuplot -persist");
    f(x) = m*x + b
    fit f(x) ’filename’ using 1:2 via m,b
    plot ’filename’ using 1:2 title ’Carrier Pigeon Delivery’ with points, f(x) title ’Model Fit’
    set arrow from $x1,$y1 to $x1,$y2 nohead lc rgb \'red\'\n"
*/