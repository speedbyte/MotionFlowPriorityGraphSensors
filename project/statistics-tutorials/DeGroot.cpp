
/**
 Demands for utility: A contractor is building an office complex and needs to plan water and electricity demands. He
 needs to hence decide the size of pipes, conduit and wires. After consulting the prospective tenants and examining
 historic data, the contractor decides tha thte demand for electricity will range from 1 million and 150 milion KWh
 per day and water demand will be between 4 and 200 ( in thousands of gallons per day ). All combinations of water
 and electricity and water demand are considered possible.

*/

#include <gnuplot/gnuplot-iostream.h> // this depends on the boost library. Also adds
#include <iostream>
#include <array>




int main ( int argc, char *argv[]) {
    // Sample space for the actual water demand and the electricity demand. The sample space has infinitely many
    // points. Indeed the sample space is uncountable.
    Gnuplot gp;

    //xy_pts_array.push_back(std::make_pair(realx[i],dm.at<float>(i)));
    //gp << "set xrange [4:200]\nset yrange [1:150]\n";
    //gp << "plot '-' with points title 'demand\n";
}

/*    for(const auto& s: xy_pts_array)
        std::cout << xy_pts_array->operator[](s) << ' ';
*/
//int *xy_ptr_array[10] = {xy_pts};    // Array of 10 pointer to integers
