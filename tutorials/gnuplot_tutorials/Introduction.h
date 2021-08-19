
/**
    gp(std::fopen("plot.gnu"));
    gp("tee plot.gnu | gnuplot -persist");
    f(x) = m*x + b
    fit f(x) ’filename’ using 1:2 via m,b
    plot ’filename’ using 1:2 title ’Carrier Pigeon Delivery’ with points, f(x) title ’Model Fit’
    set arrow from $x1,$y1 to $x1,$y2 nohead lc rgb \'red\'\n"

    Gnuplot gp(stdout) just shows what would be sent to the file.
    // I use temporary files rather than stdin because the syntax ends up being easier when
    // plotting several datasets.  With the stdin method you have to give the full plot
    // command, then all the data.  But I would rather give the portion of the plot command for
    // the first dataset, then give the data, then the command for the second dataset, then the
    // data, etc.

*/