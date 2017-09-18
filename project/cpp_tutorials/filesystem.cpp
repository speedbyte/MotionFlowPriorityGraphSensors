
#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"


#include <sys/stat.h>
#include <cstdio>
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include <cinttypes>
#include <iostream>
#include <boost/filesystem/operations.hpp>


void printing() {
    // printf
    // sprintf
    // vsnprintf
    const char *fmt = "sqrt(2) = %f";
    int sz = std::snprintf(nullptr, 0, fmt, std::sqrt(2));
    std::vector<char> buf(sz + 1); // note +1 for null terminator
    std::snprintf(&buf[0], buf.size(), fmt, std::sqrt(2));
    std::printf("printing buffer %s\n", buf);

    std::printf("Strings:\n");

    const char* s = "Hello";
    std::printf("\t[%10s]\n\t[%-10s]\n\t[%*s]\n\t[%-10.*s]\n\t[%-*.*s]\n",
                s, s, 10, s, 4, s, 10, 4, s);

    std::printf("Characters:\t%c %%\n", 65);

    std::printf("Integers\n");
    std::printf("Decimal:\t%i %d %.6i %i %.0i %+i %u\n", 1, 2, 3, 0, 0, 4, -1);
    std::printf("Hexadecimal:\t%x %x %X %#x\n", 5, 10, 10, 6);
    std::printf("Octal:\t%o %#o %#o\n", 10, 10, 4);

    std::printf("Floating point\n");
    std::printf("Rounding:\t%f %.0f %.32f\n", 1.5, 1.5, 1.5);
    std::printf("Padding:\t%05.2f %.2f %5.2f\n", 1.5, 1.5, 1.5);
    std::printf("Scientific:\t%E %e\n", 1.5, 1.5);
    std::printf("Hexadecimal:\t%a %A\n", 1.5, 1.5);
    std::printf("Special values:\t0/0=%g 1/0=%g\n", 0.0/0.0, 1.0/0.0);

    std::printf("Variable width control:\n");
    std::printf("right-justified variable width: '%*c'\n", 5, 'x');
    int r = std::printf("left-justified variable width : '%*c'\n", -5, 'x');
    std::printf("(the last printf printed %d characters)\n", r);

    // fixed-width types
    std::uint32_t val = std::numeric_limits<std::uint32_t>::max();
    std::printf("Largest 32-bit value is %" PRIu32 " or %#" PRIx32 "\n", val, val);
}

void filesystem() {
    struct stat s1;
    int abcd = 0;
    const char* path = RAW_DATASET_PATH;
    int status = stat(path, &s1);
    //std::printf("Path is a directory",abcd);
    std::printf("Path is a directory : %d \n",S_ISDIR(s1.st_mode));
}

void boost_filesystem() {
    boost::filesystem::path curdir = boost::filesystem::current_path();
    std::cout << curdir.generic_string() << std::endl << curdir.string() << std::endl << curdir
              << std::endl;
    for (const auto& dir : curdir) {
        std::cout << '[' << dir.string() << ']'; // each part
    }
}

void boost_program_options() {

/*    cv::CommandLineParser parser(argc, argv, "{@input||}{help h||}");
    std::string input = parser.get<std::string>("@input");

    if (input.size() == 1 && isdigit(input[0]))
        cap.open(input[0] - '0');
    else
        cap.open(input);
*/

}




int main()
{
    printing();
    boost_filesystem();

    return 0;
}