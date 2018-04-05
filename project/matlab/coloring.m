% Optical Flow Color Coder using Kitti Devkit

    addpath(genpath('matlab')); %path to Kitti Devkit
    input = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_snow_high/'
    output = '/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_snow_high/'

    name = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_flow_gt.png');
    temp = imread(name);
    flow = flow_read(name);
    res = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_color_gt.png');
    t = flow_to_color(flow,20);
    imwrite(t,res); 

    
    END = 3
    for x=0:END
        
    name = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_flow_algo_%01d.png',x);
    temp = imread(name);
    t1 = temp(:,:,1);
    flow = flow_read(name);
    f1 = flow(:,:,1);
    f2 = flow(:,:,2);
    f3 = flow(:,:,3);
    res = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_color_algo_%01d.png',x);
    t = flow_to_color(flow,20);
    
    imwrite(t,res);
    end

    