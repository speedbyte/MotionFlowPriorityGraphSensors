//
// Created by veikas on 23.02.18.
//

#ifndef MAIN_INPUTOUTPUT_H
#define MAIN_INPUTOUTPUT_H

namespace MotionFlow {


    class InputOutput {

    };

    class PropertyTree {

public:
        getValue() {
            boost::property_tree::ptree pt;
            boost::property_tree::read_ini("../input.txt", pt);
            try {
                std::strcmp(pt.get<std::string>("VIRES_DATASET.PLOT").c_str(), "0") == 0 ? vires_dataset.plot = false :
                        vires_dataset.plot = true;

            }
            catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::property_tree
            ::ptree_bad_path> >) {
                std::cerr << "Corrupt config file\n";
                throw;
            }
        }
    };
};
#endif //MAIN_INPUTOUTPUT_H
