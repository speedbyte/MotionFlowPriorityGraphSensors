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
