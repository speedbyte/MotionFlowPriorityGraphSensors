function [  ] = evaluation( name,noise )
%Kitti Plot, Error calculation and mean error calculation

if strcmp(name, 'slow')
    load('InitializeSlow.mat');
    load('collisionVectorSlow.mat');
    if noise == 0
        path = 'slow/flow_occ';
        gtName = 'slow';
    end
    
    if noise == 1
        path = 'slow/flow_occ_dynamic_BG';
        gtName = 'slow';
    end

    if noise == 2
        path = 'slow/flow_occ_static_BG';
        gtName = 'slow';

    end
    if noise == 3
        path = 'slow/flow_occ_static_FG';
        gtName = 'slow';
        
    end
    
    if noise == 4
        path = 'slow/flow_occ_dynamic_FG';
        gtName = 'slow';

    end
    
       

end

if strcmp(name,'normal')
    load('InitializeNormal.mat');
        load('collisionVectorNormal.mat');

        if noise == 0
        path = 'normal/flow_occ';
        gtName = 'normal';

    end
    
    if noise == 1
        path = 'normal/flow_occ_dynamic_BG';
        gtName = 'normal';

    end

    if noise == 2
        path = 'normal/flow_occ_static_BG';
        gtName = 'normal';

    end
    if noise == 3
        path = 'normal/flow_occ_static_FG';
        gtName = 'normal';

    end
    
    if noise == 4
        path = 'normal/flow_occ_dynamic_FG';
        gtName = 'normal';

    end
    
end

if strcmp(name,'fast')
    load('InitializeFast.mat');
        load('collisionVectorFast.mat');

    if noise == 0
        path = 'fast/flow_occ';
        gtName = 'fast';

    end
    
    if noise == 1
        path = 'fast/flow_occ_dynamic_BG';
        gtName = 'fast';

    end

    if noise == 2
        path = 'fast/flow_occ_static_BG';
        gtName = 'fast';

    end
    if noise == 3
        path = 'fast/flow_occ_static_FG';
        gtName = 'fast';

    end
    
    if noise == 4
        path = 'fast/flow_occ_dynamic_FG';
        gtName = 'fast';

    end
    
    
end




error = 0;
for x = 1:maxIteration
    name_flow = sprintf('./../../../../matlab_dataset/results/FB/%s/%06d_10.png',path,x);


    if x > 1
    tau = [3 0.05];
    gT = sprintf('./../../../../matlab_dataset/data/stereo_flow/flow_occ_%s/%06d_10.png',gtName,x); %GT
    addpath(genpath('../../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
    F_gt = flow_read(gT);
    F_est = flow_read(name_flow);
    f_err = flow_error(F_gt,F_est,tau);
    f_err = f_err*100;
    error(x) = f_err;
    F_err = flow_error_image(F_gt,F_est,tau);
    errSum = sum(error);
    errorMean(x) = errSum/x;

       kittiPlotter(error,f_err,errorMean,F_est,F_gt,F_err,x);
    
        disp(x);
  end
    
end


end

