%Kitti Plot, Error calculation and mean error calculation
load('Initialize.mat');


error = 0;
for x = 1:maxIteration
    name_flow = sprintf('./../../../matlab_dataset/results/FB/data/%06d_10.png',x);


    if x > 1
    tau = [3 0.05];
    name = sprintf('./../../../matlab_dataset/data/stereo_flow/flow_occ/%06d_10.png',x);
    addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
    F_gt = flow_read(name);
    F_est = flow_read(name_flow);
    f_err = flow_error(F_gt,F_est,tau);
    f_err = f_err*100;
    error(x) = f_err;
    F_err = flow_error_image(F_gt,F_est,tau);
    errSum = sum(error);
    errorMean(x) = errSum/x;

    if x < 10 || x > maxIteration - 10
       kittiPlotter(error,f_err,errorMean,F_est,F_gt,F_err,x);
    end
        disp(x);
  end
    
end