

#include <vector>
#include <iostream>
#include <fstream>
#include <boost/serialization/access.hpp>
#include <boost/archive/text_oarchive.hpp>


class BaseSaveFile {

private:
    std::vector<float> first_vector;

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize ( Archive &ar, const unsigned int version ) {
        for ( auto x:first_vector)
            ar & x;
    }

public:
    void fill_vector(std::vector<float> fill) {
        first_vector = fill;
    }

    void show_vector() {
        for ( auto x: first_vector )
        std::cout << x << std::endl;
    }

    std::vector<float> get_vector() {
        return first_vector;
    }

};


class DerivedSaveFile : public BaseSaveFile {

};


int main ( int argc, char **argv) {

    DerivedSaveFile derived;

    std::vector<float> fill;
    for ( auto i = 0; i < 10; i++) {
        fill.push_back(i);
    }

    derived.fill_vector(fill);
    derived.show_vector();

    std::ofstream save_object("../save_object.bin", std::ios::out | std::ios::binary);

    //save_object.write((char*)fill.data(), ((fill.size())*sizeof(float)));
    //save_object.write((char*)(derived.get_vector()).data(), ((derived.get_vector()).size())*sizeof(float));

    //save data to archive
    boost::archive::text_oarchive oa(save_object);
    oa << derived;


}

