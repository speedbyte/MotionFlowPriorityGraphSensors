//
// Created by veikas on 27.01.18.
//

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <vires-interface/Common/viRDBIcd.h>
#include <sys/time.h>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "ViresObjects.h"
using namespace std::chrono;

void GroundTruthScene::prepare_directories() {


    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" +  m_environment + "/";

    if (!m_datasetpath.string().compare(CPP_DATASET_PATH) || !m_datasetpath.string().compare(VIRES_DATASET_PATH)) {

        std::cout << "prepare gt_scene directories" << std::endl;

        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + m_generatepath.string()).c_str());
        }
        boost::filesystem::create_directories(m_generatepath);

        char char_dir_append[20];
        boost::filesystem::path path;

        for (int i = 1; i <= m_list_objects.size(); i++) {

            sprintf(char_dir_append, "%02d", i);
            m_trajectory_obj_path = m_generatepath.string() + "trajectory_obj_";
            path = m_trajectory_obj_path.string() + char_dir_append;
            boost::filesystem::create_directories(path);

        }

    }
}


void GroundTruthSceneInternal::generate_gt_scene(void) {

    prepare_directories();

    // Trajectories
    MyTrajectory myTrajectory2;
    MyTrajectory myTrajectory1;
    MyTrajectory myTrajectory;

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)473.976471, (float)270.655273));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)102.874649, (float)270.165527));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)472.788788, (float)270.661530));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)104.154800, (float)270.177063));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)470.727112, (float)270.672699));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)106.380035, (float)270.197113));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)467.789917, (float)270.688568));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)109.546974, (float)270.225616));







    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)464.244263, (float)266.779846));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)113.650085, (float)270.262573));







    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)457.123108, (float)266.823456));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)124.236877, (float)270.357880));









    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)453.567688, (float)266.845093));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)129.773560, (float)270.407684));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)446.474365, (float)266.888550));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)140.791489, (float)270.506836));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)442.932709, (float)266.910095));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)146.273010, (float)270.556152));









    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)439.399628, (float)266.931885));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)151.736343, (float)270.605286));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)435.867188, (float)266.953369));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)157.181549, (float)266.721680));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)432.343201, (float)266.975098));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)162.614746, (float)266.777283));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)428.819916, (float)266.996552));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)168.023773, (float)266.832428));


    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)425.301270, (float)267.017944));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)173.415100, (float)266.887390));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)421.791077, (float)267.039581));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)178.788651, (float)266.942200));








    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)418.281433, (float)267.060944));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)184.144516, (float)266.996796));


    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)414.780365, (float)267.082520));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)189.482788, (float)267.051208));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)411.279785, (float)267.103821));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)194.803406, (float)267.105469));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)407.783813, (float)267.125092));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)200.106781, (float)267.159515));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)404.296387, (float)267.146606));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)205.398468, (float)267.213684));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)400.809357, (float)267.167816));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)210.667160, (float)267.267365));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)397.330963, (float)267.189270));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)215.918716, (float)267.320923));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)393.852997, (float)267.210419));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)221.169876, (float)267.374268));








    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)390.379395, (float)267.231567));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)226.419769, (float)267.427460));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)383.449860, (float)267.273987));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)236.810440, (float)267.533325));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)379.994019, (float)267.295288));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)241.982361, (float)267.586212));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)376.538239, (float)267.316345));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)247.132263, (float)267.638702));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)373.086914, (float)267.337341));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)252.265442, (float)267.691040));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)369.644348, (float)267.358551));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)257.382263, (float)267.743195));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)366.201843, (float)267.379517));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)262.482666, (float)267.795197));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)362.768250, (float)267.400665));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)267.566711, (float)267.847015));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)359.334534, (float)267.421570));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)272.634460, (float)267.898682));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)355.905243, (float)267.442413));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)277.690887, (float)267.950409));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)352.484802, (float)267.463501));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)282.726318, (float)268.001740));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)349.064331, (float)267.484314));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)287.745697, (float)268.052917));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)345.648132, (float)267.505096));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)292.749146, (float)268.103912));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)342.240845, (float)267.526093));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)297.736603, (float)268.154755));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)338.833374, (float)267.546814));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)302.708344, (float)268.205414));







    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)307.664337, (float)268.255920));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)332.036072, (float)267.588440));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)312.609253, (float)268.306519));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)328.641602, (float)267.609100));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)317.533997, (float)268.356689));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)325.256104, (float)267.630005));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)322.443115, (float)268.406738));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)321.870239, (float)267.650574));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)327.336884, (float)268.456604));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)318.493469, (float)267.671387));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)332.215271, (float)268.506348));










    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)315.116180, (float)267.691956));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)337.078308, (float)268.555908));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)311.743164, (float)267.712463));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)341.926208, (float)268.605286));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)308.379211, (float)267.733215));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)346.763306, (float)268.654785));










    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)305.014862, (float)267.753662));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)351.580872, (float)268.703888));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)301.659485, (float)267.774353));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)356.383453, (float)268.752808));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)298.303558, (float)267.794769));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)361.171051, (float)268.801636));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)294.951843, (float)267.815155));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)365.943848, (float)268.850250));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)291.609253, (float)267.835754));


    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)370.701843, (float)268.898743));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)288.266113, (float)267.856110));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)375.445068, (float)268.947083));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)284.932007, (float)267.876648));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)380.177734, (float)268.995483));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)281.597168, (float)267.896942));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)384.891724, (float)269.043518));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)278.266571, (float)267.917175));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)389.591064, (float)269.091431));









    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)274.945129, (float)267.937683));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)394.276062, (float)269.139160));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)271.622925, (float)267.957886));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)398.946594, (float)269.186737));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)268.310059, (float)267.978302));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)403.602844, (float)269.234192));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)264.996155, (float)267.998474));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)408.244873, (float)269.281494));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)261.686432, (float)268.018585));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)412.872589, (float)269.328644));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)258.385956, (float)268.038940));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)417.490051, (float)269.375854));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)255.084549, (float)268.059021));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)422.089539, (float)269.422729));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)251.792389, (float)268.079315));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)426.675110, (float)269.469452));







    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)248.499252, (float)268.099335));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)431.246765, (float)269.516052));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)245.210205, (float)268.119354));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)435.804443, (float)269.562469));








    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)241.930527, (float)268.139587));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)440.348389, (float)269.608765));







    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)238.649841, (float)268.159515));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)444.878601, (float)269.654938));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)235.378403, (float)268.179688));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)449.398529, (float)269.701172));







    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)232.105804, (float)268.199585));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)453.901367, (float)269.747040));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)228.837265, (float)268.219482));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)458.390594, (float)269.792755));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)225.578140, (float)268.239563));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)462.866394, (float)273.404022));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)222.317734, (float)268.259399));

    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)467.328735, (float)273.444153));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)219.061417, (float)268.279205));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)471.777649, (float)273.484131));









    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)215.814514, (float)268.299225));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)476.213318, (float)273.523987));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)212.566299, (float)268.318970));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)480.638916, (float)273.563934));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)209.327682, (float)268.338928));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)206.087524, (float)268.358643));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)202.851395, (float)268.378326));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)199.624756, (float)272.135468));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)196.396652, (float)272.152802));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)193.178070, (float)272.170288));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));




    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)189.957977, (float)272.187561));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));





    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));






    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));



    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));

    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));


    myTrajectory1.pushTrajectoryPoints(cv::Point2f((float)187.650085, (float)272.200043));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f((float)484.867523, (float)273.601898));


    /*
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(50,25));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(100,50));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(150,75));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(200,100));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(250,125));

    myTrajectory2.pushTrajectoryPoints(cv::Point2f(700,250));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(800,225));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(900,200));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(1000,175));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(1100,150));
*/
    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
        myTrajectory1.pushTrajectoryPoints(points);
        myTrajectory2.pushTrajectoryPoints(points);
    }
*/
    //std::cout << myTrajectory1.getTrajectory();

    Achterbahn achterbahn1, achterbahn2;
    achterbahn1.process(Dataset::getFrameSize());
    //achterbahn1.setDynamic();
    achterbahn2.process(Dataset::getFrameSize());
    //achterbahn2.setDynamic();

    //Rectangle rectangle1(5, 5); // width, height
    Rectangle rectangle2(20,20); // width, height
    //Rectangle myShape(5, 5); // width, height
    //Circle circle;
    //Ramp ramp;
    //NegativeRamp negativeRamp;

    ColorfulNoise colorfulNoise;
    NoNoise noNoise;

    //GroundTruthObjects obj1(rectangle1, achterbahn1, 120, noNoise, "rectangle_wide");
    GroundTruthObjects obj2(rectangle2, myTrajectory1, 0, noNoise, "rectangle_long");
    GroundTruthObjects obj3(rectangle2, myTrajectory2, 0, noNoise, "random_object");

    //GroundTruthObjects obj3(rectangle, ramp, 120, noNoise, "rectangle_wide");
    //GroundTruthObjects obj4(rectangle, negativeRamp, 60, colorfulNoise, "rectangle_long");
    //GroundTruthObjects obj5(rectangle, circle, 60, colorfulNoise, "rectangle_long");

    //list_of_gt_objects.push_back(obj1);
    m_list_objects.push_back(obj2);
    m_list_objects.push_back(obj3);

    //list_of_gt_objects.push_back(obj3);
    //list_of_gt_objects.push_back(obj4);
    //list_of_gt_objects.push_back(obj5);
    /*
     * First create an object with an image_data_and_shape
     * Then define the object trajectory
     * Then copy the object image_data_and_shape on the object trajectory points
     * Then store the image in the ground truth image folder
     *
     * Then extrapolate the object trajectory with the above image_data_and_shape
     *
     * Then store the flow information in the flow folder
     */

    //m_shapes.process();
    //m_trajectories.process(Dataset::getFrameSize());

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthImage.channels() == 3);

    cv::Mat tempGroundTruthTrajectory;
    tempGroundTruthTrajectory.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthTrajectory.channels() == 3);
    tempGroundTruthTrajectory = cv::Scalar::all(255);

    cv::Mat tempGroundTruthTrajectory_2;
    tempGroundTruthTrajectory_2.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthTrajectory_2.channels() == 3);
    tempGroundTruthTrajectory_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate_single_scene_image",0},{"generate_all_scene_image", 0}};

    std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << std::endl;

    char file_name_image[50];

    cv::Mat image_data_and_shape;
    cv::Mat trajectoryShape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.

    auto tic_all = steady_clock::now();

    for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

        auto tic = steady_clock::now();

        sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
        std::string input_image_file_with_path = m_generatepath.string() + file_name_image;

        tempGroundTruthImage = m_canvas.getImageShapeAndData().get().clone();

        //draw new ground truth image.

        char frame_skip_folder_suffix[50];

        for ( unsigned  i = 0; i < m_list_objects.size(); i++ ) {

            sprintf(frame_skip_folder_suffix, "%02d", m_list_objects.at(i).getObjectId());
            std::string trajectory_image_file_with_path = m_trajectory_obj_path.string() +
                    frame_skip_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_objects.at(i).getImageShapeAndData().get().clone();
            trajectoryShape = m_list_objects.at(i).getImageShapeAndData().get().clone();

            if ( ( m_list_objects.at(i).get_obj_base_visibility().at(frame_count))
                    ) {
                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                 .getTrajectory().at(frame_count).x),
                                 cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                         .getTrajectory().at(frame_count).y), image_data_and_shape.cols,
                                 image_data_and_shape.rows)));


                if (m_list_objects.at(i).getObjectId() == 1) {
                    trajectoryShape = cv::Scalar(255, 0, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at
                                    (i).getTrajectoryPoints().getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory);
                }

                if (m_list_objects.at(i).getObjectId() == 2) {
                    trajectoryShape = cv::Scalar(0, 255, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory_2(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                                                                                                               .getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory_2);
                }
            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        auto toc = steady_clock::now();
        time_map["generate_single_scene_image"] = duration_cast<milliseconds>(toc - tic).count();

    }

    auto toc_all = steady_clock::now();
    time_map["generate_all_scene_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth scene generation time - " << time_map["generate_all_scene_image"] << "ms" << std::endl;

}

void GroundTruthSceneExternal::generate_gt_scene() {

    prepare_directories();

    char command[1024];

    std::string project = "Movement";

    std::vector<std::string> list_of_scenarios = {"carTypesComplete.xml", "crossing8Demo.xml", "crossing8DualExt.xml",
            "crossing8Static.xml", "HighwayPulk.xml", "invisibleCar.xml", "ParkPerp.xml", "RouteAndPathShapeSCP.xml",
            "staticCar.xml", "TownActionsPath.xml", "TownPathLong.xml", "traffic_demo2Ext.xml",
            "trafficDemoClosePath.xml", "trafficDemoPath.xml", "trafficDemoPed.xml", "traffic_demoReverse.xml",
            "trafficDemoTrailer.xml", "trafficDemoUK.xml", "traffic_demo.xml"
                    "car.xml", "moving_car_near.xml", "moving_car.xml", "moving_truck.xml", "moving.xml", "one.xml",
            "truck.xml", "two.xml"};

    sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(), project.c_str());
    std::cout << command << std::endl;
    system(command);

    std::cout << " I am out of bash" << std::endl;

    sleep(5); // Give some time before you send SCP commands.

    // std::string m_server;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;

    // initalize the server variable
    std::string serverName = "127.0.0.1";

    setServer(serverName.c_str());

    fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", mShmKey, getCheckMask(),
            getForceBuffer());

    bool connected_trigger_port = false;
    bool connected_module_manager_port = false;
    bool connected_scp_port = false;

    int scpSocket = openNetwork(SCP_DEFAULT_PORT);
    std::cout << "scp socket - " << scpSocket << std::endl;
    if (scpSocket != -1) { // this is blocking until the network has been opened
        connected_scp_port = true;
    }

    sleep(1); // Give some time before you send the next SCP command.

    sendSCPMessage(scpSocket, apply.c_str());

    sleep(5); // This is very important !! Mimimum 5 seconds of wait, till you start the simulation

    sendSCPMessage(scpSocket, project_name.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, rdbtrigger_portnumber.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, scenario_name.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, module_manager.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, camera_parameters.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, display_parameters.c_str());

    sleep(2);

    sendSCPMessage(scpSocket, m_environment_scp_message.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, message_scp.c_str());

    sleep(1);

    //sendSCPMessage(scpSocket, popup_scp.c_str());

    //sleep(1);

    sendSCPMessage(scpSocket, eyepoint.c_str());

    sleep(1);


    sendSCPMessage(scpSocket, elevation.c_str());

    sleep(1);

    sprintf(command, "cd %s../../ ; bash vtdRunScp.sh", (m_datasetpath.string()).c_str());
    std::cout << command << std::endl;
    system(command);

    sleep(10);  // Give some time before you start the trigger and module manager ports.

    //readScpNetwork(scpSocket);

    //readScpNetwork(scpSocket);

    //sleep(1);


    // open the network connection to the taskControl (so triggers may be sent)
    fprintf(stderr, "creating network connection....\n");
    int triggerSocket = openNetwork(DEFAULT_PORT);
    std::cout << "trigger socket - " << triggerSocket << std::endl;
    if (triggerSocket != -1) { // this is blocking until the network has been opened
        connected_trigger_port = true;
    }

    int moduleManagerSocket = openNetwork(DEFAULT_RX_PORT);
    std::cout << "mm socket - " << moduleManagerSocket << std::endl;
    if (moduleManagerSocket != -1) { // this is blocking until the network has been opened
        connected_module_manager_port = true;
    }

    if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {
        // open the shared memory for IG image output (try to attach without creating a new segment)
        fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", mShmKey);

        while (!getShmPtr()) {
            openShm(mShmKey);
            usleep(1000);     // do not overload the CPU
        }

        // now check the SHM for the time being
        bool breaking = false;
        int count = 0;

        try {
            while (1) {

                // Break out of the loop if the user presses the Esc key
                /*
                int c = kbhit();

                switch (c) {
                    case 9:
                        breaking = true;
                        break;
                    default:
                        break;
                } */

                if (breaking) {
                    break;
                }

                if (mSimFrame > MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC) {
                    breaking = true;
                }

                int lastSimFrame = mLastNetworkFrame;

                readNetwork(moduleManagerSocket);  // this calls parseRDBMessage() in vires_common.cpp

                if (lastSimFrame < 0) {
                    checkShm();  //empty IG buffer of spurious images
                }

                bool haveNewFrame = (lastSimFrame != mLastNetworkFrame);

                // now read IG output
                if (mHaveImage)
                    fprintf(stderr, "main: checking for IG image\n");

                while (mCheckForImage) {
                    checkShm();

                    mCheckForImage = !mHaveImage;

                    usleep(10);

                    if (!mCheckForImage) {
                        //fprintf( stderr, "main: got it!\n" );
                    }
                }

                if (haveNewFrame) {
                    //fprintf( stderr, "main: new simulation frame (%d) available, mLastIGTriggerFrame = %d\n",
                    //                 mLastNetworkFrame, mLastIGTriggerFrame );

                    mHaveFirstFrame = true;
                }

                // has an image arrived or do the first frames need to be triggered
                //(first image will arrive with a certain image_02_frame delay only)


                if (!mHaveFirstImage || mHaveImage || haveNewFrame || !mHaveFirstFrame) {
                    // do not initialize too fast
                    if (!mHaveFirstImage || !mHaveFirstFrame)
                        usleep(100000);   // 10Hz

                    bool requestImage = (mLastNetworkFrame >= (mLastIGTriggerFrame + IMAGE_SKIP_FACTOR_DYNAMIC));

                    if (requestImage) {
                        mLastIGTriggerFrame = mLastNetworkFrame;
                        mCheckForImage = true;
                    }

                    //fprintf( stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = %s\n", mDeltaTime,
                    //         requestImage ? "true" : "false" );
                    sendRDBTrigger(triggerSocket, mSimTime, mSimFrame, requestImage, mDeltaTime);

                    // increase internal counters
                    mSimTime += mDeltaTime;
                    mSimFrame++;

                    // calculate the timing statistics
                    if (mHaveImage)
                        //calcStatistics();

                        mHaveImage = false;
                }

                usleep(10000); // sleep for 10 ms
                //std::cout << "getting data from VIRES\n";
            }
        }
        catch (...) {
            sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << "End of generation" << std::endl;
            return;
        };

        try {

            Noise noNoise;
            Rectangle myShape(40,40);
            GroundTruthObjects character(myShape, myTrajectoryVector.at(0), 0, noNoise, "New Character");
            GroundTruthObjects character_01(myShape, myTrajectoryVector.at(1), 0, noNoise, "New Character01");

            m_list_objects.push_back(character);
            m_list_objects.push_back(character_01);

        }
        catch (...)
        {
            sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << "End of generation" << std::endl;

        }

    }

    sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
    std::cout << command << std::endl;
    system(command);
    std::cout << "End of generation" << std::endl;

}


void GroundTruthSceneExternal::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    //fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    //fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {

    mLastNetworkFrame = simFrame;

    //fprintf(stderr, "headers %d\n,", RDB_PKG_ID_END_OF_FRAME);
    //fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

/** ------ state of an object (may be extended by the next structure) ------- */
typedef struct
{
    uint32_t            id;                         /**< unique object ID                                              @unit _                                   */
    uint8_t             category;                   /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink  */
    uint8_t             type;                       /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink  */
    uint16_t            visMask;                    /**< visibility mask                                               @unit @link RDB_OBJECT_VIS_FLAG @endlink  */
    char                name[RDB_SIZE_OBJECT_NAME]; /**< symbolic name                                                 @unit _                                   */
    RDB_GEOMETRY_t      geo;                        /**< info about object's geometry                                  @unit m,m,m,m,m,m                         */
    RDB_COORD_t         pos;                        /**< position and orientation of object's reference point          @unit m,m,m,rad,rad,rad                   */
    uint32_t            parent;                     /**< unique ID of parent object                                    @unit _                                   */
    uint16_t            cfgFlags;                   /**< configuration flags                                           @unit @link RDB_OBJECT_CFG_FLAG @endlink  */
    int16_t             cfgModelId;                 /**< visual model ID (configuration parameter)                     @unit _                                   */
} RDB_OBJECT_STATE_BASE_DUMMY_t;

/** ------ extended object data (e.g. for dynamic objects) ------- */
typedef struct
{
    RDB_COORD_t         speed;                      /**< speed and rates                                               @unit m/s,m/s,m/s,rad/s,rad/s,rad/s           */
    RDB_COORD_t         accel;                      /**< acceleration                                                  @unit m/s2,m/s2,m/s2,rad/s2,rad/s2/rad/s2     */
    float               traveledDist;               /**< traveled distance                                             @unit m                                      a */
    uint32_t            spare[3];                   /**< reserved for future use                                       @unit _                                       */
} RDB_OBJECT_STATE_EXT_DUMMY_t;

/** ------ sensor definition and state ------ */
typedef struct
{
    uint32_t    id;                          /**< id of the sensor                                      @unit _                                      */
    uint8_t     type;                        /**< type of the sensor                                    @unit @link RDB_SENSOR_TYPE     @endlink     */
    uint8_t     hostCategory;                /**< category of the object hosting the sensor             @unit @link RDB_OBJECT_CATEGORY @endlink     */
    uint16_t    spare0;                      /**< for future use                                        @unit _                                      */
    uint32_t    hostId;                      /**< unique id of the sensor's host                        @unit _                                      */
    char        name[RDB_SIZE_OBJECT_NAME];  /**< name of the sensor                                    @unit _                                      */
    float       fovHV[2];                    /**< field-of-view (horizontal/vertical)                   @unit rad,rad                                */
    float       clipNF[2];                   /**< clipping ranges (near/far)                            @unit m,m                                    */
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  @unit m,m,m,rad,rad,rad                      */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     @unit m,m,m,rad,rad,rad                      */
    float       fovOffHV[2];                 /**< field-of-view offset (horizontal/vertical)            @unit rad, rad                              B */
    int32_t     spare[2];                    /**< for future use                                        @unit _                                      */
} RDB_SENSOR_STATE_DUMMY_t;

/** ------ information about an object registered within a sensor ------ */
typedef struct
{
    uint8_t     category;    /**< object category                                                                @unit @link RDB_OBJECT_CATEGORY    @endlink   */
    uint8_t     type;        /**< object type                                                                    @unit @link RDB_OBJECT_TYPE        @endlink   */
    uint16_t    flags;       /**< sensor object flags                                                            @unit @link RDB_SENSOR_OBJECT_FLAG @endlink   */
    uint32_t    id;          /**< id of the object                                                               @unit _                                       */
    uint32_t    sensorId;    /**< id of the detecting sensor                                                     @unit _                                       */
    double      dist;        /**< distance between object and referring device                                   @unit m                                       */
    RDB_COORD_t sensorPos;   /**< position and orientation of object in sensor coord                             @unit m,m,m,rad,rad,rad                       */
    int8_t      occlusion;   /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)    @unit [-1, 0..127]                            */
    uint8_t     spare0[3];   /**< for future use                                                                 @unit _                                       */
    uint32_t    spare[3];    /**< for future use                                                                 @unit _                                       */
} RDB_SENSOR_OBJECT_DUMMY_t;

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_CFG_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    RDB_OBJECT_CFG_t *object = reinterpret_cast<RDB_OBJECT_CFG_t *>(data); /// raw image data
    std::cout << object->type;

}

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_STATE_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned
                                          short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {

    RDB_OBJECT_STATE_t *object = reinterpret_cast<RDB_OBJECT_STATE_t *>(data); /// raw image data
//        fprintf( stderr, "handleRDBitem: handling object state\n" );
//        fprintf( stderr, "    simTime = %.3lf, simFrame = %d\n", simTime, simFrame );
//        fprintf( stderr, "    object = %s, id = %d\n", data->base.name, data->base.id );
//        fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", data->base.pos.x, data->base.pos.y, data->base
//                .pos.z );
    ViresObjects viresObjects = ViresObjects();
    viresObjects.objectProperties = *object;
    viresObjects.frame_count = simFrame;

    if ( ( mSimFrame % IMAGE_SKIP_FACTOR_DYNAMIC== 0 ) && mSimFrame > 1 && data->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN) {

        fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                    data->base.name, simFrame, data->base.pos.x, object->base.pos.y, data->base.geo.dimX, data->base
                            .geo.dimY);

        printf("%d.pushTrajectoryPoints(cv::Point2f((float)%f, (float)%f))\n", data->base.id, data->base.pos.x, data->base.pos.y);
        std::cout << data->base.type;
        myTrajectoryVector.at(data->base.id-3).pushTrajectoryPoints(cv::Point2f((float)data->base.pos.x, (float)
                data->base.pos.y));
    }
}

void GroundTruthSceneExternal::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;
    //fprintf(stderr, "handleRDBitem: image\n");
    //fprintf(stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    //fprintf(stderr, "    width / height = %d / %d\n", data->width, data->height);
    //fprintf(stderr, "    dataSize = %d\n", data->imgSize);

    // ok, I have an image:

    //analyzeImage(  data  , simFrame, 0 );
    mHaveImage      = true;
    mHaveFirstImage = true;

    fprintf( stderr, "------------------------------------------------------------------------------------\n");
    fprintf( stderr, "simFrame = %d, simTime = %.3f, dataSize = %d\n", mSimFrame, mSimTime, data->imgSize);
    fprintf( stderr, "------------------------------------------------------------------------------------\n");

    char *image_data_ = NULL;
    RDB_IMAGE_t *image = reinterpret_cast<RDB_IMAGE_t *>(data); /// raw image data

    /// RDB image information of \see image_data_
    RDB_IMAGE_t image_info_;
    memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

    if (NULL == image_data_) {
        image_data_ = reinterpret_cast<char *>(malloc(image_info_.imgSize));
    } else {
        image_data_ = reinterpret_cast<char *>(realloc(image_data_, image_info_.imgSize));
    }
    // jump data header
    memcpy(image_data_, reinterpret_cast<char *>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

    if (image_info_.imgSize == image_info_.width * image_info_.height * 3) {
        png::image<png::rgb_pixel> save_image(image_info_.width, image_info_.height);
        unsigned int count = 0;
        for (int32_t v = 0; v < image_info_.height; v++) {
            for (int32_t u = 0; u < image_info_.width; u++) {
                png::rgb_pixel val;
                val.red = (unsigned char) image_data_[count++];
                val.green = (unsigned char) image_data_[count++];
                val.blue = (unsigned char) image_data_[count++];
                //val.alpha = (unsigned char)image_data_[count++];
                save_image.set_pixel(u, v, val);
            }
        }

        //fprintf(stderr, "got a RGB image with %d channels\n", image_info_.imgSize / (image_info_.width * image_info_
        //.height));

        char file_name_image[50];


        if (simFrame > 1) {
            sprintf(file_name_image, "000%03d_10.png", mImageCount);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;
            save_image.write(input_image_file_with_path);
            mImageCount++;
        }
    } else {
        fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize / (image_info_
                                                                                           .width *
                                                                                   image_info_.height));
    }
}

double GroundTruthSceneExternal::getTime()
{
    struct timeval tme;
    gettimeofday(&tme, 0);

    double now = tme.tv_sec + 1.0e-6 * tme.tv_usec;

    if ( mStartTime < 0.0 )
        mStartTime = now;

    return now;
}


void GroundTruthSceneExternal::calcStatistics()
{
    double now = getTime();

    double dt = now - mStartTime;

    if ( dt < 1.e-6 )
        return;

    fprintf( stderr, "calcStatistics: received %d/%d images in %.3lf seconds (i.e. %.3lf/%.3lf images per second ), total number of errors = %d\n",
             mTotalNoImages, dt, mTotalNoImages / dt, mTotalErrorCount );
}

void GroundTruthSceneExternal::analyzeImage( RDB_IMAGE_t* img, const unsigned int & simFrame, unsigned int index )
{
    static unsigned int sLastImgSimFrame =  0;

    if ( !img || ( index > 1 ) )
        return;

    if ( img->id == 0 )
        return;

    fprintf( stderr, "analyzeImage: simframe = %d, index = %d: have image no. %d, size = %d bytes, pixelFormat = %d\n",
             simFrame, index, img->id, img->imgSize, img->pixelFormat );

    if ( img->pixelFormat == RDB_PIX_FORMAT_RGB32F )		// some analysis
    {
        float *imgData = ( float* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %.3f / %.3f / %.3f\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }
    else if ( img->pixelFormat == RDB_PIX_FORMAT_RGB8 )		// some analysis
    {
        unsigned char *imgData = ( unsigned char* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %d / %d / %d\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }


    //if ( ( myImg->id > 3 ) && ( ( myImg->id - mLastImageId ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    if ( ( simFrame != sLastImgSimFrame ) && ( img->id > 3 ) && ( ( simFrame - sLastImgSimFrame ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    {
        fprintf( stderr, "WARNING: parseRDBMessageEntry: index = %d, delta of image ID out of bounds: delta = %d\n", index, simFrame - sLastImgSimFrame );
        mTotalErrorCount++;
    }

    mLastImageId    = img->id;
    mTotalNoImages++;

    sLastImgSimFrame = simFrame;
}
