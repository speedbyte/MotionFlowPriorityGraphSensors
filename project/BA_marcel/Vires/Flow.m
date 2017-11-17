%Loads the Initialization specs.



clear all;
close all;

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.004);


load('start.mat');
for x = startFrame:150
    
    disp(x);
    
    name_frame = sprintf('./../../../vires_dataset/data/stereo_flow/image_02_car/%06d_10.png',x);   %imread
    name_flow = sprintf('./../../../vires_dataset/results/FB/flow_occ_car/%06d_10.png',x-startFrame);          %result
    
    tic;
    frame = imread(name_frame);
    frame_gray = rgb2gray(frame);
    
    flow_frame = estimateFlow(opticFlow,frame_gray);
    flowstop(x) = toc;
    
    
    vxCopy = (abs(flow_frame.Vx) > 0.2);
    vyCopy = (abs(flow_frame.Vy) > 0.2);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
    res = cat(3,flow_frame.Vx,flow_frame.Vy,vCopy);
    
    
    addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
    
    flow_write(res,name_flow);
    
    
    
end
