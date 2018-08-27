//Kitti Plot, Error calculation and mean error calculation
if x > 2
tau = [3 0.05];
name = sprintf('./GroundTruth/%06d_10.png', x);
addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
F_gt = flow_read(name);
F_est = flow_read('result.png');
f_err = flow_error(F_gt, F_est, tau);
f_err = f_err * 100;
error(x) = f_err;
F_err = flow_error_image(F_gt, F_est, tau);
errSum = sum(error);
errorMean(x) = errSum / x;


plotter(frame, flow_frame, collisionVector, estimatedCollisionVector, actualX, actualY, secondActualX,
        secondActualY, estMovement, x, timeToGenerateObject, flowstop, plotTime, collisionTime, timeMovement,
        error, f_err, errorMean, F_est, F_gt, F_err);

end


int main() {
//Object specification

    //ushort XMovement=0,YMovement=0,secondXMovement=0,secondYMovement=0;
    std::vector<bool> estimatedCollisionVector(MAX_ITERATION);
    cv::Mat absoluteFlow = cv::Mat::zeros(frame_size, CV_16UC3);
    cv::Mat flow1 = cv::Mat::zeros(frame_size, CV_16UC3);
    cv::Mat flow2 = cv::Mat::zeros(frame_size, CV_16UC3);

    ushort actualX;
    ushort actualY;
    ushort secondActualX;
    ushort secondActualY;

    //object specs
    const ushort width = 30;
    const ushort height = 100;

    //ushort xOrigin = xPos.at(start);
    //ushort yOrigin = yPos.at(start);

    //ushort secondXOrigin = xPos.at(secondstart);
    //ushort secondYOrigin = yPos.at(secondstart);

    //for moving the objects later
    //actualX = xOrigin;
    //actualY = yOrigin;
    //secondActualX = secondXOrigin;
    //secondActualY = secondYOrigin;

    //Initialization
    //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement
    iterator = 0;
    sIterator = 0;
    //XMovement = 0;
    //secondXMovement = 0;
    //YMovement = 0;
    //secondYMovement = 0;


    std::vector<ushort> XSpec;
    for (ushort i = 0; i < width; i++) {
        XSpec.push_back(actualX + i);
    }
    std::vector<ushort> YSpec;
    for (ushort i = 0; i < height; i++) {
        YSpec.push_back(actualY + i);
    }

    std::vector<ushort> secondXSpec;
    for (ushort i = 0; i < width; i++) {
        secondXSpec.push_back(secondActualX + i);
    }

    std::vector<ushort> secondYSpec;
    for (ushort i = 0; i < height; i++) {
        secondYSpec.push_back(secondActualY + i);
    }

//Scratch 2

//create flow matrix to store the estimated displacemend in.

//absolute Flow - get absolute estimated flow.


    bool estimatedCollision;
    std::vector<float> estMovement(4); // xMean,yMean,secondObjectXMean,secondObjectYMean


//Threshold of the object flow
    estimatedCollision = 0;
    cv::Mat flowFirstObjectX = cv::Mat::zeros(frame_size, CV_16UC1);
    cv::Mat flowFirstObjectY = cv::Mat::zeros(frame_size,CV_16UC1);
    cv::Mat flowSecondObjectX = cv::Mat::zeros(frame_size,CV_16UC1);
    cv::Mat flowSecondObjectY = cv::Mat::zeros(frame_size,CV_16UC1);

    ushort upperheigtht = YSpec.at(YSpec.size()-1);
    ushort lowerheight = YSpec.at(0);
    ushort upperwidth = XSpec.at(XSpec.size()-1);
    ushort lowerwidth = XSpec.at(0);

    ushort secondObjectUpperheight = secondYSpec.at(YSpec.size()-1);
    ushort secondObjectLowerheight = secondYSpec.at(0);
    ushort secondObjectUpperWidth = secondXSpec.at(XSpec.size()-1);
    ushort secondObjectLowerWidth = secondXSpec.at(0);


    for ( int k = YSpec.at(0); k < YSpec.at(YSpec.size()-1); k++ )  {
        for ( int j = XSpec.at(0); j < XSpec.at(XSpec.size()-1); j++ )  {
            flow1.at<cv::Vec3i>(k, j)[0] = absoluteFlow.at<cv::Vec3i>(YSpec.at(k), XSpec.at(j))[0];
            flow1.at<cv::Vec3i>(k, j)[1] = absoluteFlow.at<cv::Vec3i>(YSpec.at(k), XSpec.at(j))[1];
        }
    }

    for ( int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size()-1); k++ )  {
        for ( int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size()-1); j++ )  {
            flow2.at<cv::Vec3i>(k, j)[0] = absoluteFlow.at<cv::Vec3i>(secondYSpec.at(k), secondXSpec.at(j))[0];
            flow2.at<cv::Vec3i>(k, j)[1] = absoluteFlow.at<cv::Vec3i>(secondYSpec.at(k), secondXSpec.at(j))[1];
        }
    }

//%
//%get flow from the objects
    for ( ushort k = lowerheight; k < upperheigtht; k++ ) {
        for ( ushort j = lowerwidth; j < upperwidth; j++ ) {
            flowFirstObjectX.at<ushort>(k,j) = flow.at<cv::Vec3i>(k,j)[0];
            flowFirstObjectY.at<ushort>(k,j) = flow.at<cv::Vec3i>(k,j)[1];
        }
    }

    for ( ushort kk = secondObjectLowerheight; kk < secondObjectUpperheight; kk++ ) {
        for ( ushort jj = secondObjectLowerWidth; jj < secondObjectUpperWidth; jj++ ) {
            flowSecondObjectX.at<ushort>(kk,jj) = flow.at<cv::Vec3i>(kk,jj)[0];
            flowSecondObjectY.at<ushort>(kk,jj) = flow.at<cv::Vec3i>(kk,jj)[1];
        }
    }

//%
//Extract the movement of the object.
    cv::Mat firstObjectX, firstObjectY, secondObjectX, secondObjectY;

    for ( ushort i = 0; i < flowFirstObjectX.rows; i++ ) {
        for (ushort j = 0; j < flowFirstObjectX.cols; i++) {
            if ( flowFirstObjectX.at<ushort>(i,j) != 0 || cv::abs(flowFirstObjectX.at<ushort>(i,j) < 0.2)) {
                firstObjectX.push_back(flowFirstObjectX.at<ushort>(i,j));
            }
        }
    }

    for ( ushort i = 0; i < flowFirstObjectY.rows; i++ ) {
        for (ushort j = 0; j < flowFirstObjectY.cols; i++) {
            if ( flowFirstObjectY.at<ushort>(i,j) != 0 || cv::abs(flowFirstObjectY.at<ushort>(i,j) < 0.2)) {
                firstObjectY.push_back(flowFirstObjectY.at<ushort>(i,j));
            }
        }
    }

    for ( ushort i = 0; i < flowSecondObjectX.rows; i++ ) {
        for (ushort j = 0; j < flowSecondObjectX.cols; i++) {
            if ( flowSecondObjectX.at<ushort>(i,j) != 0 || cv::abs(flowSecondObjectX.at<ushort>(i,j) < 0.2)) {
                secondObjectX.push_back(flowSecondObjectX.at<ushort>(i,j));
            }
        }
    }

    for ( ushort i = 0; i < flowSecondObjectY.rows; i++ ) {
        for (ushort j = 0; j < flowSecondObjectY.cols; i++) {
            if ( flowSecondObjectY.at<ushort>(i,j) != 0 || cv::abs(flowSecondObjectY.at<ushort>(i,j) < 0.2)) {
                secondObjectY.push_back(flowSecondObjectY.at<ushort>(i,j));
            }
        }
    }

    cv::Scalar_<float> xMean, yMean, secondObjectXMean, secondObjectYMean;

//Get the movement by getting the mean of the flow objects.

    xMean = cv::mean(firstObjectX);
    yMean = cv::mean(firstObjectY);
    secondObjectXMean = cv::mean(secondObjectX);
    secondObjectYMean = cv::mean(secondObjectY);

    std::cout << "Estimated Movement of the First Object:" << std::endl;;
    std::cout << "x" << std::endl;
    std::cout << "xMean" << std::endl;
    std::cout << "y" << std::endl;
    std::cout << "yMean" << std::endl;
    std::cout << "Estimated Movement of the second Object" << std::endl;;
    std::cout << "x" << std::endl;
    std::cout << "secondObjectXMean" << std::endl;
    std::cout << "y" << std::endl;
    std::cout << "secondObjectYMean" << std::endl;

//Estimate the future collision. Floor call in order to get possibly matching results


    for ( ushort i = 0; i < XSpec.size(); i++ ) {
        XSpec.at(i) = std::floor(XSpec.at(i) + xMean[0]);
        secondXSpec.at(i) = std::floor(secondXSpec.at(i) + secondObjectXMean[0]);
    }

    for ( ushort i = 0; i < YSpec.size(); i++ ) {
        YSpec.at(i) = std::floor(YSpec.at(i) + yMean[0]);
        secondYSpec.at(i) = std::floor(secondYSpec.at(i) + secondObjectYMean[0]);
    }

    std::vector<ushort> checkX, checkY;
    std::vector<ushort> collisionVector(MAX_ITERATION);

    for ( ushort i = 0; i < XSpec.size() ; i++) {
        for ( ushort j = 0; j < secondXSpec.size() ; j++) {
            if ( std::abs(XSpec.at(i) - secondXSpec.at(j)) == 0) {
                checkX.push_back(i);  // index of collision
            }
        }
    }
    for ( ushort i = 0; i < YSpec.size() ; i++) {
        for ( ushort j = 0; j < secondYSpec.size() ; j++) {
            if ( std::abs(YSpec.at(i) - secondYSpec.at(j)) == 0) {
                checkY.push_back(i);  // index of collision
            }
        }
    }

    if (!checkX.empty() && !checkY.empty()) {
        collisionVector.at(x) = 1;
    }
    else {
        collisionVector.at(x) = 0;
    }


    estMovement.at(0) = xMean[0];
    estMovement.at(1) = yMean[0];
    estMovement.at(2) = secondObjectXMean[0];
    estMovement.at(3) = secondObjectYMean[0];


    for ( int k = 0; k < YSpec.size(); k++ )  {
        for ( int j = 0; j < XSpec.size(); j++ ) {
            absoluteFlow.at<cv::Vec3i>(k, j)[0] = estMovement.at(0) + j;
            absoluteFlow.at<cv::Vec3i>(k, j)[1] = estMovement.at(1) + k;
            absoluteFlow.at<cv::Vec3i>(k, j)[2] = 1;
        }
    }
    for ( int k = 0; k < secondYSpec.size(); k++ )  {
        for ( int j = 0; j < secondXSpec.size(); j++ ) {
            absoluteFlow.at<cv::Vec3i>(k, j)[0] = estMovement.at(2) + j;
            absoluteFlow.at<cv::Vec3i>(k, j)[1] = estMovement.at(3) + k;
            absoluteFlow.at<cv::Vec3i>(k, j)[2] = 1;
        }
    }
//abs1 = absoluteFlow(:,:,1);


////collision checkers(first round has bad flow, dont plot and estimate collision
//time to check for collision

    if (estimatedCollision == 1)
        estimatedCollisionVector.at(x) = 1;
    else
        estimatedCollisionVector.at(x) = 0;

    cv::Mat vxCopy, vyCopy, vXYCopy, vCopy, flow_frame_individual_channels[3];

    cv::split(flow_frame, flow_frame_individual_channels);
    //flow_frame.Vx ~ = 0); // flow_frame != 0 ? 1 : 0
    cv::threshold (flow_frame_individual_channels[0], vxCopy, 0, 1, cv::THRESH_BINARY);
    cv::threshold (flow_frame_individual_channels[1], vyCopy, 0, 1, cv::THRESH_BINARY);
    vXYCopy = vxCopy + vyCopy;
    cv::threshold (vXYCopy, vCopy, 0, 1, cv::THRESH_BINARY);

    cv::Mat res;

    cv::Mat flow;
    flow_frame_individual_channels->copyTo(flow);

    //channel copy !!!!!!
    flow_frame_individual_channels[0] = (flow_frame_individual_channels[0] * 64 ) + 2 ^ 15; // Vx
    flow_frame_individual_channels[1] = (flow_frame_individual_channels[1] * 64 ) + 2 ^ 15; // Vy
    flow_frame_individual_channels[2] = vCopy;

    // merge in a image file
    cv::merge(flow_frame_individual_channels,3,res);

// Scratch 2 ends



// Scratch absolute
    for ( int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size()-1); k++ )  {
        for ( int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size()-1); j++ )  {
            absoluteGroundTruth.at<cv::Vec3w>(k,j)[0] = (ushort)(secondXMovement+j);
            absoluteGroundTruth.at<cv::Vec3w>(k,j)[1] = (ushort)(secondYMovement+k);
            absoluteGroundTruth.at<cv::Vec3w>(k,j)[2] = 1;
        }
    }




    if ((iterator + start) >= xPos.size()) {
        start = 0;
        iterator = 0;
    }

    if ((sIterator + secondstart) >= xPos.size()) {
        secondstart = 0;
        sIterator = 0;
    }

    iterator++;
    sIterator++;

    //Update position (the objects of interest are tracked via Ground Truth here)

    actualX = xPos.at(start + iterator);
    actualY = yPos.at(start + iterator);
    secondActualX = xPos.at(secondstart + sIterator);
    secondActualY = yPos.at(secondstart + sIterator);

    cv::imwrite(name_frame.string(), frame );



    /* Scratch GT */
    //check for each frame (iteration) if the objects are colliding

        std::vector<ushort> xCol = XSpec;
        std::vector<ushort> yCol = YSpec;

        std::vector<ushort> secondXCol= secondXSpec;
        std::vector<ushort> secondYCol = secondYSpec;

        std::vector<ushort> checkX, checkY;
        std::vector<ushort> collisionVector;
        for ( ushort i = 0; i < MAX_ITERATION; i++) {
            collisionVector.push_back(0);
        }

        for ( ushort i = 0; i < xCol.size() ; i++) {
            for ( ushort j = 0; j < secondXCol.size() ; j++) {
                if ( std::abs(xCol.at(i) - secondXCol.at(j)) == 0) {
                    checkX.push_back(i);  // index of collision
                    break;
                }
            }
        }
        for ( ushort i = 0; i < yCol.size() ; i++) {
            for ( ushort j = 0; j < secondYCol.size() ; j++) {
                if ( std::abs(yCol.at(i) - secondYCol.at(j)) == 0) {
                    checkY.push_back(i);  // index of collision
                    break;
                }
            }
        }

        if (!checkX.empty() && !checkY.empty()) {
            collisionVector.at(x) = 1;
        }
        else {
            collisionVector.at(x) = 0;
        }

        /* Scratch GT ends*/

}


void calcBBFrom3DPosition(int screen_width, int screen_height, QVector3D cam_pos, float fov_v, float pixSize = 2.2e-6){
    std::vector<QVector3D> bounding_points_3d;
    //all 8 3d bounding box points of an object
    //VTD center of object with z = 0!
    bounding_points_3d.push_back(QVector3D(  m_realworld_dim.length/2,  m_realworld_dim.width/2, 0));
    bounding_points_3d.push_back(QVector3D( -m_realworld_dim.length/2,  m_realworld_dim.width/2, 0));
    bounding_points_3d.push_back(QVector3D(  m_realworld_dim.length/2, -m_realworld_dim.width/2, 0));
    bounding_points_3d.push_back(QVector3D( -m_realworld_dim.length/2, -m_realworld_dim.width/2, 0));
    bounding_points_3d.push_back(QVector3D(  m_realworld_dim.length/2,  m_realworld_dim.width/2, m_realworld_dim.height));
    bounding_points_3d.push_back(QVector3D( -m_realworld_dim.length/2,  m_realworld_dim.width/2, m_realworld_dim.height));
    bounding_points_3d.push_back(QVector3D(  m_realworld_dim.length/2, -m_realworld_dim.width/2, m_realworld_dim.height));
    bounding_points_3d.push_back(QVector3D( -m_realworld_dim.length/2, -m_realworld_dim.width/2, m_realworld_dim.height));

    std::vector<QPoint> bounding_points_2d;

    qreal width = screen_width, height = screen_height;
    qreal fovv = fov_v / 180. * M_PI; // [rad]
    qreal distToImagePlane = 0.5 * height / tan(fovv/2); // [px]
    qreal pxSize = pixSize; // [m/px]
    QVector3D toMeter = QVector3D(pxSize, pxSize, 1);
    qreal z = distToImagePlane * pxSize; // [m]

    QPoint min(2000,2000), max(0,0);
    //transformation matrix to transform to camera location
    QMatrix4x4 toCamPos;
    toCamPos.translate(-cam_pos); //translate camera pos


    //iterate over bounding points and add transformed points to path to print
    for(QVector3D p: bounding_points_3d){
        //calculate correct bounding box point by adding offset to reference point (for VTD  = read middle axle of the car)
        p+=getDimensionOffset();
        QVector3D pos = getRealworldPos();

        //transformation matrix for offset to center of roi and the 3d bounding box point p
        QMatrix4x4 toPosition;
        QQuaternion rot = getRealWorldOrientation().getRotation();
        toPosition.translate(rot * (p));
        //first translate to camera pos
        pos = toCamPos * pos;
        //then translate and rotate to 3d bounding box point
        pos = toPosition * pos;

        //transform from sensor coordinates to camera coordinates
        pos = QVector3D(pos.y(), pos.z(),pos.x());

        //scale 3D point back onto image
        pos = pos * (z / pos.z());
        //convert meter to pixel
        pos = pos / toMeter;
        bounding_points_2d.push_back(QPoint(width/2 - pos.x(), height/2 - pos.y()));
    }
    //get min and max for x, y to get the correct 2d bounding box
    for(QPoint p : bounding_points_2d){
        if(p.x() < min.x()){
            min.setX(p.x());
        }
        if(p.y() < min.y()){
            min.setY(p.y());
        }
        if(p.x() > max.x()){
            max.setX(p.x());
        }
        if(p.y() > max.y()){
            max.setY(p.y());
        }
    }
    m_bb.setTopLeft(min);
    m_bb.setBottomRight(max);
}

std::vector<unsigned> x_pts;
std::vector<double> y_pts;
std::vector<unsigned> z_pts;

/*
 To summarize, we compare six costs: sampling-insensitive absolute differences (BT), three filter-based costs (LoG, Rank, and Mean), hierarchical mutual information (HMI), and normalized cross-correlation (NCC).*
 */



auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();

pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
// gnuplot_2d
Gnuplot gp2d;
gp2d << "set xrange [0:" + std::to_string(MAX_ITERATION_RESULTS) + "]\n";
gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
std::string tmp = std::string(" with points title ") + std::string("'") + Dataset::m_dataset_gtpath.string() +
                  std::string(" y axis - ms, x axis - image_02_frame\n'");
//gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;

for(auto &n : time)
sum_time +=n;




// in ground truth part
if ( datafilter_index < 0 ) {

if ( scenario_displacement_occurence.count(std::make_pair(std::round(gt_displacement.x*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF,std::round(gt_displacement.y*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF)) ) {

}
else {

}

/*
if ( scenario_displacement_occurence.count(std::make_pair(65535,65535)) ) {
    scenario_displacement_occurence[std::make_pair(65535,65535)] = scenario_displacement_occurence[std::make_pair(65535,65535)] + 1;
}
else {
    scenario_displacement_occurence[std::make_pair(65535,65535)] = 1;
}*/
}


// in else part
if ( datafilter_index < 0 ) {
if ( scenario_displacement_occurence.count(std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)) ) {
scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] = scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] + 1;
}
else {
scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] = 1;
}

/*
if ( scenario_displacement_occurence.count(std::make_pair(65535,65535)) ) {
    scenario_displacement_occurence[std::make_pair(65535,65535)] = scenario_displacement_occurence[std::make_pair(65535,65535)] + 1;
}
else {
    scenario_displacement_occurence[std::make_pair(65535,65535)] = 1;
}*/

}



std::cout << "\nMean\n" << mean << "\nCovar\n" << covar_new <<
"\nstddev_x\n" << stddev <<
"\ncorr\n" << corr << std::endl;


void AlgorithmFlow::store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
&l_pixelmovement )  {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  l_pixelposition.y << "col" << l_pixelposition.x << "displacement" << "[:";
    fs << l_pixelmovement.x;
    fs << l_pixelmovement.y;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}



void scratch() {


    cv::VideoWriter video_out;

    if ( frame_types == video_frames)
    {
        boost::filesystem::path video_out_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + "movement.avi" ;
        assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
//frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
//frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
        video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, Dataset::m_frame_size);
        printf("Writer eingerichtet\n");
    }


    if ( frame_types == video_frames) {
        cv::VideoCapture cap;
        cap.open(Dataset::m_dataset_gtpath.string() + "image_02/movement.avi");
        if (!cap.isOpened()) {
            std::cout << "Could not initialize capturing...\n";
            return;
        }
    }

    if ( frame_types == video_frames) {
        video_out.write(image_02_frame);
    }

    if ( frame_types == video_frames) {
        video_out.release();
    }

}


void run_optical_flow_algorithm() {

    // TODO scratch : if stencil size does not work

    if ( new_stencil_size == 0 ) {
        for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
            for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                cv::Point2f algo_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(current_frame_index).second;
                //std::cout << roi_offset.x + col_index << std::endl;
                stencil_movement.at(obj_index).push_back(
                        std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                       algo_displacement));
                base_visibility.at(obj_index).push_back(visibility);
                F_png_write.setFlowU(roi_offset.x + col_index,roi_offset.y + row_index, algo_displacement.x);
                F_png_write.setFlowV(roi_offset.x + col_index,roi_offset.y + row_index, algo_displacement.y);
                F_png_write.setValid(roi_offset.x + col_index,roi_offset.y + row_index, true);

            }
        }
        std::cout << stencil_movement.at(obj_index).size() << "hacked" << next_pts_array.size() << std::endl;
    }


    new_stencil_size = stencil_movement.at(obj_index).size();
    assert(new_stencil_size != 0);



    // tested - norm is the same as displacement calculation using formula
    auto dist_ = cv::norm(displacement);
    double dist;
    dist = pow(displacement.x,2)+pow(displacement.y,2); //calculating distance by euclidean formula
    dist = sqrt(dist);
    assert(dist==dist_);


}


/*
png::image<png::rgb_pixel> color_image(image_info_.width, image_info_.height);
unsigned int count = 0;
for (int32_t v = 0; v < image_info_.height; v++) {
    for (int32_t u = 0; u < image_info_.width; u++) {
        png::rgb_pixel val;
        val.red = (unsigned char) image_data_[count++];
        val.green = (unsigned char) image_data_[count++];
        val.blue = (unsigned char) image_data_[count++];
        //val.alpha = (unsigned char)image_data_[count++];
        color_image.set_pixel(u, v, val);
    }
}
*/


roi_offset
roi_size
roi(rowrange.colrange)
roi.locateROI(roi_size, roi_offset);



for (unsigned j = 0; j < width; j += 1) {
for (unsigned k = 0; k < height; k += 1) {
temp_frame_coordinates_displacement.push_back(
        std::make_pair(cv::Point2f(columnBegin + j, rowBegin + k), gt_displacement));
//frame_stencil_visibility.push_back(visibility);
}
}

frame_stencil_displacement.resize(temp_frame_coordinates_displacement.size());
frame_stencil_visibility.resize(temp_frame_coordinates_displacement.size());

frame_stencil_displacement.clear();
frame_stencil_visibility.clear();


/*
const std::vector<std::pair<cv::Point2f,cv::Point2f > > &ground_truth = m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index);
for (unsigned row_index = (unsigned)rowBegin; row_index < rowBegin+height; row_index++) {
    for (unsigned col_index = (unsigned)columnBegin; col_index < columnBegin+width; col_index++) {

        for (ushort next_pts_index = 0;
             next_pts_index < frame_next_pts_array.size(); next_pts_index++) {
            if (((col_index) ==
                 std::round(frame_next_pts_array.at(next_pts_index).x)) &&
                ((row_index) ==
                 std::round(frame_next_pts_array.at(next_pts_index).y))) {

                cv::Point2f algo_displacement = displacement_array.at(next_pts_index);

                frame_stencil_displacement.push_back(std::make_pair(
                        cv::Point2f(col_index, row_index),
                        algo_displacement));
                frame_stencil_visibility.push_back(visibility);

            }
        }
    }
}
*/


/*
first find all moving objects in the frame through frame differencing
then find the bounding box of these moving objects
then mark the region of interest
then find the interesection of bounding box region of interest and all moving objects
then refine the intersection of region of interest using depth information.

the depth information is only unique when the objects are overlapping each other. Otherwise multiple objects can have the same depth.

finalImage can be used for presentation because it consists the occlusion boundaries.

 */

// The following code ddid not work because the ground truth objects will never have common corrdinates.
MyIntersection myIntersection_gt_object_pairs;

std::vector<std::pair<cv::Point2f, cv::Point2f>> intersection_ground_truth_objects(
        groundtruthobject1.size());
intersection_ground_truth_objects.clear();

myIntersection_gt_object_pairs.find_intersection_pair(groundtruthobject1.begin(), groundtruthobject1.end(),
        groundtruthobject2.begin(), groundtruthobject2.end(),
        intersection_ground_truth_objects.begin());
intersection_ground_truth_objects = myIntersection_gt_object_pairs.getResultPair();
//myIntersection_gt_object_pairs.showResult();
std::cout << "occlusion intersection between two objects" << intersection_ground_truth_objects.size() << std::endl;

for ( auto it = intersection_ground_truth_objects.begin(); it != intersection_ground_truth_objects.end(); it++) {
cv::circle(check_intersection, (*it).first, 1, cv::Scalar(0,255,0));
}
