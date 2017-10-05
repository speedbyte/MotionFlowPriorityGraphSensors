#!/bin/bash
rm plot.dat

writedata() {
    for i in {0..10}; do
        echo -e $i"\t"$((i*i))"\t"$i >> plot.dat
        sleep 1
        echo "done"
    done
}

writedata &
sleep 1
gnuplot liveplot.gnu