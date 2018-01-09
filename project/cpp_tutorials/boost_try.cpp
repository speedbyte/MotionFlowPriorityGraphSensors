
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <map>
#include <unordered_map>


int main()
{

    std::map<std::string,int> ordered_map =
            {
                    {"Veikas", 12},
                    {"Yumin", 34},
                    {"Michael", 56}
            };

    std::unordered_map<std::string,int> unordered_map =
            {
                    {"Veikas", 12},
                    {"Yumin", 34},
                    {"Michael", 56}
            };

    //std::cout << ordered_map["Veikas"] << '\n';
    //std::cout << unordered_map["Veikas"] << '\n';

    for (std::pair<std::string const, int> mapping : ordered_map) { // & means it is a reference to the
        // element in ordered_map. We can also use without the reference, but then any changes in mapping wont
        // reflect in the original ordered_,map.
        std::cout << mapping.first << std::endl;
        mapping.second = 33;
    }
    std::cout << ordered_map["Veikas"] << std::endl;

    for (auto& mapping : ordered_map) {
        std::cout << mapping.first << std::endl;
    }

    for ( auto i = 0; i < 10; i++) {
        printf ( "%d\n", i );
    }

    std::vector<int> vec = {1,2,3,4,5};
    for ( auto p : vec ) { // i is equivalent to vec[x]. If vec would have been a list, then it was possible to use
        // the member functions like i.memberfunction
        // type is typeof(vec)
        printf ( "auto test %d\n", p );
    }
    std::vector<int>::iterator it = vec.begin();
    for ( auto it = vec.begin(); it <= vec.end() ; it = it + 2) {
        // type is std::vector<int>::iterator
        // it is an iterator, *it is the element to which it refers, ++it advances it to refer to the next element.
        printf ( "iterator test %d\n", it.operator*() );
    }

    using boost::property_tree::ptree;
    ptree pt;
    read_ini("../input.txt", pt);

    for (auto& section : pt)
    {
        std::cout << "[!" << section.first << "]\n";
        for (auto& key : section.second)
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    }

    std::cout << pt.get<std::string>("Section1.section1_val1") << std::endl;
    std::cout << pt.get<std::string>("Section1.section1_val2") << std::endl;
}


