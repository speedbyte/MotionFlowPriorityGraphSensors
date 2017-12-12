%Calculates and nearwise accurate Ground Truth
%log file is : Frame xPos yPos xDim yDim


xFlow  = './../../../vires_dataset/data/stereo_flow/image_02_car/Flow.log'

x = fopen(xFlow);
%Fromat : Framenumber PosX PosY width height
formatSpec = '%f %f %f %f %f ';
sizeA = [5,inf];
[groundTruth ] = fscanf(x,formatSpec,sizeA);
groundTruth = groundTruth';
groundTruth = round(groundTruth);

fclose(x);

startFrame = 35;

for x = startFrame:100
    disp(x);
    
    %%For creating the GroundTruth pixels as vires only delivers the
    %%position of the object as x,y coordinate and not for each pixel.
    %%Therefore parameters need to be changed manually
    
    OverFlowHandlerX = round((groundTruth(x,2)-(0.5*groundTruth(x,4)))) : round(groundTruth(x,2)+0.5*groundTruth(x,4));
    OverFlowHandlerX(OverFlowHandlerX > 800)=[];
    
    OverFlowHandlerY = round(600-groundTruth(x,3)-(0.75*groundTruth(x,5)):600-groundTruth(x,3)+(0.48*groundTruth(x,5)));
    OverFlowHandlerY(OverFlowHandlerY > 600)=[];
    
    name_frame = zeros(600,800);
    
    for j = OverFlowHandlerY
        for k = OverFlowHandlerX
            name_frame(j,k,1) = groundTruth(x+1,2)-groundTruth(x,2);
            name_frame(j,k,2) = groundTruth(x+1,3)-groundTruth(x,3);
            name_frame(j,k,3) = 1;
        end
    end
    
    
    vxCopy = (name_frame(:,:,1) ~= 0);
    vyCopy = (name_frame(:,:,2) ~= 0);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
    res = cat(3,name_frame(:,:,1),name_frame(:,:,2),vCopy);
    
    addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
    
    % instantiate and save ground truth flow matrix png file for each iteration.
    name_flow = sprintf('./../../../vires_dataset/data/stereo_flow/flow_occ_car/%06d_10.png',x-startFrame); %Ground Truth
    flow_write(res,name_flow);
    
    save('start.mat','startFrame');
    
end

    