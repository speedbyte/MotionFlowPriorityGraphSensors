// All display functions can access Storage private variables
friend class Display;

// Only Display::displayItem can access Storage private variables.
friend void Display::displayItem(Storage& storage); // error: Storage hasn't seen the full definition of class Display

// Everyone can access Storage private variables
friend void displayItem(Storage &storage);


if(argc < 3 || argc > 5) {
std::cout << "plotvelodyne [kitti database root] [dataset] [save]" << std::endl;
exit(-1);
}
dataset /= argv[2];
dataset /= dataset.parent_path().leaf();
dataset += "_drive_";
dataset += argv[3];
dataset += "_sync/";

std::cout << "dataset: " << dataset << std::endl;
if(argc == 5)
output += argv[4];
else
output += argv[1];


.hpp




.cpp


//      All the below is to make the compiler happy.

template void Larger<int>::printLargest();
template void Larger<int>::printLarger();
template void Larger<double>::printLarger();
template Larger<int>::Larger(int);
template Larger<double>::Larger(double);
template Larger<char>::Larger(char);

Instead of doing this, once can simply do this

template class Larger<int>;
template class Larger<double>;
template class Larger<char>;



