%Loads the Initialization specs. 



clear all;
close all;

load('Initialize.mat');
load('collisionVector.mat');

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.004);





plotTime = 1;
error(1) = 0;
error(2) = 0;


    mkdir('./../../../matlab_dataset/results/FB/data/');


for x = 1:maxIteration
    
    disp(x);

    name_frame = sprintf('./../../../matlab_dataset/data/stereo_flow/image_2/%06d_10.png',x);   %imread
    name_flow = sprintf('./../../../matlab_dataset/results/FB/data/%06d_10.png',x);          %result

    

        

    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    absoluteFlow = zeros(375,1242,3,'int16');
    
   %Initialization
    if x == 1
    iterator = 0;
    sIterator = 0;
    end
    
    %end of path vector? reset
    if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end
    if sIterator+secondStart > length(xPos)
        secondStart = 1;
        sIterator = 0;
    end


    
    %%
    %Optical Flow
   tic;
   frame = imread(name_frame);
   frame_gray = rgb2gray(frame);
   
   flow_frame = estimateFlow(opticFlow,frame_gray);
   flowstop(x) = toc;
  
   
    vxCopy = (flow_frame.Vx ~= 0);
    vyCopy = (flow_frame.Vy ~= 0);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
     res = cat(3,flow_frame.Vx,flow_frame.Vy,vCopy);
     
     addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
     
     flow_write(res,name_flow);
     

         
      %create flow matrix to store the estimated displacemend in.
        flow(:,:,1) = flow_frame.Vx;
        flow(:,:,2) = flow_frame.Vy;
        
      %absolute Flow
      %get absolute estimated flow.
      
         tic;
         [estMovement,estimatedCollision] = estimatedMovement(flow, xSpec,ySpec, secondXSpec,secondYSpec);
         timeMovement(x) = toc;
         collisionTime(x) = toc;

        for k=ySpec
            for j=xSpec
                absoluteFlow(k,j,1) = estMovement(1)+j;
                absoluteFlow(k,j,2) = estMovement(2)+k;
                absoluteFlow(k,j,3) = 1;
            end
        end
        for k=secondYSpec
            for j=secondXSpec
                absoluteFlow(k,j,1) = estMovement(3)+j;
                absoluteFlow(k,j,2) = estMovement(4)+k;
                absoluteFlow(k,j,3) = 1;
            end
        end
        abs1 = absoluteFlow(:,:,1);
        
        
        %%
        %%collision checkers(first round has bad flow, dont plot and
        %%estimate collision
        %time to check for collision
        
        
       %  estimatedCollision = flowCollision(absoluteFlow, xSpec,ySpec, secondXSpec,secondYSpec);
        
       if estimatedCollision == 1
            estimatedCollisionVector(x) = 1;
       else
           estimatedCollisionVector(x) = 0;
       end
       
       
       tic;
    
%       plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,estMovement,x,flowstop,plotTime,collisionTime, timeMovement);
          plotTime(x) = toc;

    
    %%
    %Update position(the objects of interest are tracked via Ground Truth
    %here)

    actualX = xPos(start+iterator);
    actualY = yPos(start+iterator);
    secondActualX = xPos(secondStart+sIterator);
    secondActualY = yPos(secondStart+sIterator);
    
  
    iterator = iterator+1;
    sIterator = sIterator+1;
    
end
