//
// Created by veikas on 25.01.18.
//

#include <iostream>
#include <kitti/mail.h>
#include "KittiSubmitInterface.h"

extern bool eval (std::string result_sha,Mail* mail);

int KittiSubmitInterface::sendData(int argc, char *argv[]) {

    // we need 2 or 4 arguments!
    if (argc!=2 && argc!=4) {
        std::cout << "Usage: ./eval_flow result_sha [user_sha email]" << std::endl;
        return 1;
    }

    // read arguments
    std::string result_sha = argv[1];

    // init notification mail
    Mail *mail;
    if (argc==4) mail = new Mail(argv[3]);
    else         mail = new Mail();
    mail->msg("Thank you for participating in our evaluation!");

    // run evaluation
    bool success = eval(result_sha,mail);
    if (argc==4) mail->finalize(success,"flow",result_sha,argv[2]);
    else         mail->finalize(success,"flow",result_sha);

    // send mail and exit
    delete mail;
    return 0;
}

