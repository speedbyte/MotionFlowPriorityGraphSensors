% Optical Flow Color Coder using Kitti Devkit

addpath(genpath('matlab')); %path to Kitti Devkit


input = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/three/ground_truth/flow_occ_00/'
output = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/three/ground_truth/kitti_00/'

%colorFunc(input,output)


input = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/three/results_FB_blue_sky_30_2/flow_occ_00/'
output = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/three/results_FB_blue_sky_30_2/kitti_00/'

colorFunc(input,output)

input = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/three/results_FB_heavy_snow_30_2/flow_occ_00/'
output = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/three/results_FB_heavy_snow_30_2/kitti_00/'

%colorFunc(input,output)
    