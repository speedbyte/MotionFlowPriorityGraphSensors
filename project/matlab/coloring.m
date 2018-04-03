% Optical Flow Color Coder using Kitti Devkit

    addpath(genpath('matlab')); %path to Kitti Devkit
    input = '/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/stereo_flow/two/results_FB_night/'
    output = '/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/stereo_flow/two/results_FB_night/'
    END = 3
    for x=0:END
        
    name = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/stereo_flow/two/results_FB_night/stencil/000003_10_flow_algo_%01d.png',x);
    temp = imread(name);
    t1 = temp(:,:,1);
    flow = flow_read(name);
    f1 = flow(:,:,1);
    f2 = flow(:,:,2);
    f3 = flow(:,:,3);
    res = sprintf('/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/stereo_flow/two/results_FB_night/stencil/000003_10_color_algo_%01d.png',x);
    t = flow_to_color(flow);
    
    imwrite(t,res);
    end
