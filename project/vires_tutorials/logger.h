//
// Created by veikas on 12.09.17.
//

#ifndef VIRES_TUTORIAL_LOGGER_H_H
#define VIRES_TUTORIAL_LOGGER_H_H

#include <boost/filesystem/path.hpp>

namespace adsim {
    namespace utility {
        class Logger {
        public:
            Logger(boost::filesystem::path) {}
            void error(std::string errorLog) {
                std::cout << errorLog;
            }
        };

        static Logger GetLogger(boost::filesystem::path pathLogger) {
            return Logger(pathLogger);
        };
    }
}

#endif //VIRES_TUTORIAL_LOGGER_H_H
