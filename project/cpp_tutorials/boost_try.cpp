
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using boost::property_tree::ptree;

int main()
{

    for ( auto i = 0; i < 10; i++) {
        printf ( "%d\n", i );
    }

    std::iterator it;
    for ( auto i : it ) {
        printf ( "%d\n", i );
    }

    ptree pt;
    read_ini("../input.txt", pt);

    for (auto& section : pt)
    {
        std::cout << "[!" << section.first << "]\n";
        for (auto& key : section.second)
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    }

    //boost::property_tree::ini_parser::read_ini("config.ini", pt);
    std::cout << pt.get<std::string>("Section1.section1_val1") << std::endl;
    std::cout << pt.get<std::string>("Section1.section1_val2") << std::endl;
}


