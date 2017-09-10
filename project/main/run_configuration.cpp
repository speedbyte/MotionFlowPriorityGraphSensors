


#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>

typedef boost::filesystem::path boost_path;


boost::filesystem::path
get_file(const boost_path &dataset_path, const boost_path &subfolder, const boost_path &file_name) {

    boost::filesystem::path file_path;

    file_path += dataset_path;
    file_path += subfolder;
    file_path += file_name;

    try {
        if ( !boost::filesystem::exists(file_path) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << file_path.string() << ":" <<  exception;
        exit(0);
    }

    return file_path;

}
