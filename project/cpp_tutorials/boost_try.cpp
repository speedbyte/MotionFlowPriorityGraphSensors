
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <map>
#include <unordered_map>

typedef struct message {
    unsigned int a = 0;
    char c[32];
} STRUCT_MESSAGE;

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

    STRUCT_MESSAGE msg_write, msg_read;

    std::ofstream myfile = std::ofstream("file.bin", std::ios::out | std::ios::binary);
    memset(&msg_write, 0x0, sizeof(msg_write));
    msg_write.a = 0xAA;
    sprintf(msg_write.c, "first");
    myfile.write((char *)&msg_write, sizeof(msg_write));
    msg_write.a = 0xBB;
    sprintf(msg_write.c, "second");
    myfile.write((char *)&msg_write, sizeof(msg_write));
    myfile.close();

    std::ifstream myfile_in;
    myfile_in = std::ifstream("file.bin", std::ios::binary);
    myfile_in.read((char *)&msg_read, sizeof(msg_write));
    printf("0x%X", msg_read.a);
    myfile_in.read((char *)&msg_read, sizeof(msg_write));
    printf("0x%X", msg_read.a);
    myfile_in.close();

    //std::cout << msg2.a;

}



