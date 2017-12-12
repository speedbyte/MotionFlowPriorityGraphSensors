%Kitti Plot, Error calculation and mean error calculation
%load('Initialize.mat');

%Evaluation
clear all;

load('start.mat');

for x = 1:100
    %FLow, index is x+1
    name_flow = sprintf('./../../../vires_dataset/results/FB/flow_occ_car/%06d_10.png',x+1);
    %shown image, add startFrame to show the right one
    img = sprintf('./../../../vires_dataset/data/stereo_flow/image_02_car/%06d_10.png',x+startFrame);
    %GT FLOW Index is x
    gtFlow = sprintf('./../../../vires_dataset/data/stereo_flow/flow_occ_car/%06d_10.png',x);
    
    
     
    addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
    img_gt = imread(img);
    F_est = flow_read(name_flow);
    color = flow_to_color(F_est,4);
    F_gt = flow_read(gtFlow);
    val = F_est(:,:,3);
    valGT = F_gt(:,:,3);
    %counter of correct Flow pixels
    counter = 0;
    for i=1:600
        for j=1:800
            if val(i,j) && valGT(i,j)
                img_gt(i,j,1) = color(i,j,1);
            end
            
            if i > 1 && j > 1
                if (val(i,j) && valGT(i,j) == 1) || (val(i-1,j) && valGT(i,j) == 1) || (val(i,j-1) && valGT(i,j) == 1) || (val(i-1,j-1) && valGT(i,j) == 1)
                    if  (abs(F_gt(i,j,1) - F_est(i,j,1)) < 1) %&& (abs(F_gt(i,j,2) - F_est(i,j,2)) < 1)
                        counter = counter+1;
                        
                    end
                end
            end
            
        end
    end
    
    pixCount(x+1) = counter;
    
    
    
    
    viresPlotter(img_gt,pixCount,x,color);
    
    
    
    
    disp(sprintf('Mean %2f',sum(pixCount)/x+1));
    disp(x);
    
    
end