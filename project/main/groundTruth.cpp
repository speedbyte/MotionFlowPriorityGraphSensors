

#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <png++/png.hpp>

#include <kitti/mail.h>
#include <kitti/io_flow.h>

#include "datasets.h"
//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

extern void flow(std::string results_sha, ushort start, ushort secondstart);
//extern bool eval(std::string result_sha, Mail *mail);
extern void plotVectorField (FlowImage &F,std::string dir,char* prefix);

void ground_truth(ushort start=60, ushort secondstart=240) {

    cv::Size_<unsigned> frame_size(1242,375);
    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};
    boost::filesystem::path gt_image_path, gt_flow_path, gt_video_path;
    gt_image_path = std::string(CPP_DATASET_PATH) + std::string("data/stereo_flow/image_0/dummy.txt");
    assert(boost::filesystem::exists(gt_image_path.parent_path()) != 0);
    gt_flow_path = std::string(CPP_DATASET_PATH) + std::string("data/stereo_flow/flow_occ/dummy.txt");
    assert(boost::filesystem::exists(gt_flow_path.parent_path()) != 0);
    gt_video_path = std::string(CPP_DATASET_PATH) + std::string("data/stereo_flow/image_0/gtMovement.avi");
    assert(boost::filesystem::exists(gt_video_path.parent_path()) != 0);

    cv::VideoWriter video_out;
    video_out.open(gt_video_path.string(), CV_FOURCC('D', 'I', 'V', 'X'), 30, frame_size, true);

    //how many interations(frames)?
    auto tic= steady_clock::now();
    auto toc= steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    const ushort MAX_ITERATION = 360;
    ushort iterator = 0, sIterator = 0;
    std::vector<ushort> xPos, yPos;

    short XMovement,YMovement,secondXMovement,secondYMovement;

    ushort actualX;
    ushort actualY;
    ushort secondActualX;
    ushort secondActualY;

    //object specs
    const ushort width = 30;
    const ushort height = 40;

    std::vector<ushort> theta;
    for ( ushort x = 0; x < MAX_ITERATION; x++) {
        theta.push_back(x);
    }

    for ( int i = 0; i< MAX_ITERATION; i++) {
        xPos.push_back(static_cast<ushort>((frame_size.width/2) + (500 * cos(theta[i] * CV_PI / 180.0) /
                (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        yPos.push_back(static_cast<ushort>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));
    }

    ushort xOrigin = xPos.at(start);
    ushort yOrigin = yPos.at(start);

    ushort secondXOrigin = xPos.at(secondstart);
    ushort secondYOrigin = yPos.at(secondstart);

    //for moving the objects later
    actualX = xOrigin;
    actualY = yOrigin;
    secondActualX = secondXOrigin;
    secondActualY = secondYOrigin;

    //Initialization
    //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement
    iterator = 0;
    sIterator = 0;
    XMovement = 0;
    secondXMovement = 0;
    YMovement = 0;
    secondYMovement = 0;

    cv::Mat relativeGroundTruth(frame_size,CV_32FC3,cv::Scalar(0,0,0));

    cv::Mat test_frame = cv::Mat::zeros(frame_size, CV_8UC3);

    test_frame = cv::Scalar((rand()%255),(rand()%255),0);
    char file_name[20], file_name_gp[20];

    tic_all = steady_clock::now();
    for (ushort x=0; x < MAX_ITERATION; x++) {

        //Used to store the GT images for the kitti devkit

        relativeGroundTruth = cv::Scalar::all(0);
        sprintf(file_name, "000%03d_10", x);
        std::string gt_image_path_str = gt_image_path.parent_path().string() + "/" + std::string(file_name) + ".png";
        std::string gt_flow_path_str = gt_flow_path.parent_path().string() + "/" + std::string(file_name) + ".png";
        printf("%u, %u , %u, %u, %u, %u, %u, %i, %i\n", x, start, iterator, secondstart, sIterator, actualX, actualY,
               XMovement, secondXMovement);

        //If we are at the end of the path vector, we need to reset our iterators
        if ((iterator+start) >= xPos.size()) {
            start = 0;
            iterator = 0;
            XMovement = xPos.at(0) - xPos.at(xPos.size() - 1);
            YMovement = yPos.at(0) - yPos.at(yPos.size() - 1);
        } else {
            XMovement = xPos.at(start+iterator) - xPos.at(start+iterator-(ushort)1);
            YMovement = yPos.at(start+iterator) - yPos.at(start+iterator-(ushort)1);
        }

        if ((sIterator+secondstart) >= xPos.size()) {
            secondstart = 0;
            sIterator = 0;
            secondXMovement = xPos.at(0) - xPos.at(xPos.size()-1);
            secondYMovement = yPos.at(0) - yPos.at(yPos.size()-1);
        } else {
            secondXMovement = xPos.at(secondstart+sIterator) - xPos.at(secondstart+sIterator-(ushort)1);
            secondYMovement = yPos.at(secondstart+sIterator) - yPos.at(secondstart+sIterator-(ushort)1);
        }

        //Object specification
        std::vector<ushort> XSpec;
        for ( ushort i = 0; i < width; i++) {
            XSpec.push_back(actualX+i);
        }

        std::vector<ushort> YSpec;
        for ( ushort i = 0; i < height; i++) {
            YSpec.push_back(actualY+i);
        }

        std::vector<ushort> secondXSpec;
        for ( ushort i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX+i);
        }

        std::vector<ushort> secondYSpec;
        for ( ushort i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY+i);
        }


        // create the frame
        tic = steady_clock::now();

        uchar r = 0;
        uchar b = 0;

        //reset the image to white
        test_frame = cv::Scalar::all(255);



        //draw new image.
        for (int k = YSpec.at(0); k < YSpec.at(YSpec.size() - 1); k++) {
            for (int j = XSpec.at(0); j < XSpec.at(XSpec.size() - 1); j++) {
                test_frame.at<cv::Vec3b>(k, j)[0] = b;
                test_frame.at<cv::Vec3b>(k, j)[1] = 0;
                test_frame.at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        r = 0;
        b = 0;

        //expand with 2nd Object
        //draw new image.
        for (int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size() - 1); k++) {
            for (int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size() - 1); j++) {
                test_frame.at<cv::Vec3b>(k, j)[0] = b;
                test_frame.at<cv::Vec3b>(k, j)[1] = 0;
                test_frame.at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        r = 0;
        b = 0;

        if ( !video_out.isOpened() ) {
            std::cerr << "Could not open video" << std::endl;
        }
        cv::imwrite(gt_image_path_str, test_frame);
        video_out.write(test_frame);

        toc = steady_clock::now();
        time_map["generate"] =  duration_cast<milliseconds>(toc - tic).count();


        // calculating the relative Ground Truth for the Kitti devkit and store it in a png file
        // Displacements between -512 to 512 are allowed. Smaller than -512 and greater than 512 will result in an
        // overflow. The final value to be stored in U16 in the form of val*64+32768

        assert(relativeGroundTruth.channels() == 3);

        cv::Mat roi;
        roi = relativeGroundTruth.
                colRange(XSpec.at(0), XSpec.at(XSpec.size()-1)).
                rowRange(YSpec.at(0), YSpec.at(YSpec.size()-1));

        // store displacement in the matrix
        roi = cv::Scalar((XMovement), (YMovement), 1.0f);

        roi = relativeGroundTruth.
                colRange(secondXSpec.at(0), secondXSpec.at(secondXSpec.size()-1)).
                rowRange(secondYSpec.at(0), secondYSpec.at(secondYSpec.size()-1));

        // store displacement in the matrix
        roi = cv::Scalar((secondXMovement), (secondYMovement), 1.0f);

        //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
        FlowImage F_gt(frame_size.width, frame_size.height);
        for (int32_t v=0; v<frame_size.height; v++) { // rows
            for (int32_t u=0; u<frame_size.width; u++) {  // cols
                if (relativeGroundTruth.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                    F_gt.setFlowU(u,v,relativeGroundTruth.at<cv::Vec3f>(v,u)[0]);
                    F_gt.setFlowV(u,v,relativeGroundTruth.at<cv::Vec3f>(v,u)[1]);
                    F_gt.setValid(u,v,(bool)relativeGroundTruth.at<cv::Vec3f>(v,u)[2]);
                }
            }
        }
        F_gt.write(gt_flow_path_str);

        //plotVectorField (F_gt,gt_image_path.parent_path().string(),file_name);

        iterator++;
        sIterator++;

        actualX = actualX + XMovement;
        actualY = actualY + YMovement;
        secondActualX = secondActualX + secondXMovement;
        secondActualY = secondActualY + secondYMovement;

        std::cout << "generate frame - " << time_map["generate"]  << "ms" << std::endl;

    }
    video_out.release();
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}

std::string prepare_directories(std::string result_sha) {
    std::string result_dir = "results/" + result_sha;
    boost::filesystem::create_directories(std::string(CPP_DATASET_PATH) + result_dir + ("/data"));
    boost::filesystem::create_directories(std::string(CPP_DATASET_PATH) + result_dir + ("/video"));
    return result_dir;
}

void test_kitti_original() {

    // read arguments
    std::string result_sha = "self_generated_OF";
    std::string user_sha = "vagrawal";

    // init notification mail
    Mail *mail;
    mail = new Mail("vagrawal@hs-esslingen.de");
    mail->msg("Thank you for participating in our evaluation!");

    // run evaluation
    //bool success = eval(result_sha,mail);
    //mail->finalize(success,"flow",result_sha,user_sha);
    // send mail and exit
    delete mail;

}

int main() {
    std::string result_dir;
    //test_kitti_original();

    //ground_truth((ushort)60,(ushort)240);

    result_dir = prepare_directories("FB");
    flow(result_dir,(ushort)60,(ushort)240);

    result_dir = prepare_directories("LK");
    flow(result_dir,(ushort)60,(ushort)240);

}