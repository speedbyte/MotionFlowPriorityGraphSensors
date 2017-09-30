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
plotTime(x) = toc = steady_clock::now();
end


//Scratch 2
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
cv::imwrite("result.png", res);

//create flow matrix to store the estimated displacemend in.

//absolute Flow - get absolute estimated flow.

tic = steady_clock::now();

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

toc = steady_clock::now();

time_map["movement"] = duration_cast<milliseconds>(toc - tic).count();
time_map["collision"] =  duration_cast<milliseconds>(toc - tic).count();



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

tic = steady_clock::now();


iterator++;
sIterator++;

//Update position (the objects of interest are tracked via Ground Truth here)

actualX = xPos.at(start + iterator);
actualY = yPos.at(start + iterator);
secondActualX = xPos.at(secondStart + sIterator);
secondActualY = yPos.at(secondStart + sIterator);

cv::imwrite(name_frame.string(), frame );

auto end = steady_clock::now();
// Scratch 2 ends