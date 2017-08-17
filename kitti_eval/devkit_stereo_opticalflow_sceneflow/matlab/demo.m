disp('======= KITTI 2015 Benchmark Demo =======');
clear all; close all; dbstop error;

% error threshold 3px, >5%
tau = [3 0.2];

% original image
orig = imread('../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/image_2/000169_10.png');
figure;
imshow(orig);
title('Original Image');

% flow demo
disp('Load and show optical flow field ... ');
%F_est = flow_read('data/flow_est.png');
%F_est = flow_read('../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/flow_noc/000169_10.png');

%F_gt  = flow_read('data/flow_gt.png');
F_gt  = flow_read('../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/flow_noc/000169_10.png');

%% DENSE
F_est_dense = flow_read('../../../project/BA_marcel/matlab/results_dense/000169_10.png');
f_err = flow_error(F_gt,F_est_dense,tau);
F_err = flow_error_image(F_gt,F_est_dense,tau);
figure;
imshow([flow_to_color([F_est_dense;F_gt]);F_err]);
title(sprintf('Dense Flow Error: %.2f %',f_err*100));
%%

%% SPARSE
F_est_sparse = flow_read('../../../project/BA_marcel/matlab/results_sparse/000169_10.png');
f_err = flow_error(F_gt,F_est_sparse,tau);
F_err = flow_error_image(F_gt,F_est_sparse,tau);
figure;
imshow([flow_to_color([F_est_sparse;F_gt]);F_err]);
title(sprintf('Sparse Flow Error: %.2f %',f_err*100));
%%

% Do not need 
%% Disparity 
%disp('Load and show disparity map ... ');
%%D_est = disp_read('data/disp_est.png');
%D_est = disp_read('../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/disp_occ_0/000169_10.png');
%%D_gt  = disp_read('data/disp_gt.png');
%D_gt  = disp_read('../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/disp_occ_0/000169_10.png');
%d_err = disp_error(D_gt,D_est,tau);
%D_err = disp_error_image(D_gt,D_est,tau);
%figure;
%imshow([disp_to_color([D_est;D_gt]);D_err]);
%title(sprintf('Disparity Error: %.2f %%',d_err*100));
%%
