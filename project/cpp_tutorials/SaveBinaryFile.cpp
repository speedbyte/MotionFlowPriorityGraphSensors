

#include <vector>
#include <iostream>
#include <fstream>
#include <boost/serialization/access.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


class BaseSaveFile {

protected:
    std::vector<float> first_vector;


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
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize ( Archive &ar, const unsigned int version ) {
        ar & boost::serialization::base_object<BaseSaveFile>(*this);
        for ( auto x:first_vector)
            ar & x;
    }

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

    DerivedSaveFile newg;
    // create and open an archive for input
    std::ifstream ifs("../save_object.bin", std::ios::in | std::ios::binary);
    boost::archive::text_iarchive ia(ifs);
    // read class state from archive
    ia >> newg;
    // archive and stream closed when destructors are called

    newg.show_vector();

}

