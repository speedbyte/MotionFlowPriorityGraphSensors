% Optical Flow Color Coder using Kitti Devkit

    addpath(genpath('matlab')); %path to Kitti Devkit
    
    END = 4
    for x=0:END
        
    name = sprintf('%06d_10.png',x);
    flow = flow_read(name);
    res = sprintf('flow%01d.png',x);
    t = flow_to_color(flow,1000);
    
    imwrite(t,res);
    end
