%Loads the Initialization specs. 

%kitti einbinden
%Document 
%make everything more generic
%classify everything that moves as object
%noise(static, dynamic)

%59-70
%Err Mean 2.50880

%Noise  frame = imnoise(frame,'gaussian',0.05);
%Err Mean 4.4

%Noise  frame = imnoise(frame,'gaussian',0.05);
% Err Mean 4.76

%%%21.9
%NOISE MIGHT HAVE POSITIVE INFLUENCE



clear all;
close all;

load('Initialize.mat');
load('collisionVector.mat');

opticFlow=opticalFlowFarneback;%('NoiseThreshold',0.004);
frame = zeros(375,1242,3,'uint8');
bg = zeros(375,1242,3,'uint8');
plotTime = 1;
error(1) = 0;
error(2) = 0;

    for k=1:375
        for j=1:1242
                bg(k,j,1) = randi([0,255]);
                bg(k,j,2) = randi([0,255]);
                
                
        end
    end



for x = 1:maxIteration

    name_frame = sprintf('./Videos/Frames/%06d.png',x);
    name_flow = sprintf('./Videos/Flow/%06d.png',x);

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
        
    %Time to generate the object and frame
    tic;
    %Generate Object
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    absoluteFlow = zeros(375,1242,3,'int16');
    
  
    %create the frame
    disp(x);
    frame = movement(xSpec,ySpec,secondXSpec,secondYSpec,bg);
   %   frame = imnoise(frame,'gaussian',0.5);

    
    %add noise
    

    timeToGenerateObject(x) = toc;
    
    addpath('LKpyramid Codes');
    %%
    %Optical Flow
   tic;
   frame_gray = rgb2gray(frame);
   flow_frame = estimateFlow(opticFlow,frame_gray);
   flowstop(x) = toc;
  
   
    vxCopy = (flow_frame.Vx ~= 0);
    vyCopy = (flow_frame.Vy ~= 0);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
     res = cat(3,flow_frame.Vx,flow_frame.Vy,vCopy);
     res =  uint16(res);
     imwrite(res,'result.png');
    
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
       
    %Kitti Plot, Error calculation and mean error calculation
    if x > 2
    tau = [3 0.05];
    name = sprintf('./GroundTruth/%06d_10.png',x);
    addpath(genpath('./Kitti'));
    F_gt = flow_read(name);
    F_est = flow_read('result.png');
    f_err = flow_error(F_gt,F_est,tau);
    f_err = f_err*100;
    error(x) = f_err;
    F_err = flow_error_image(F_gt,F_est,tau);
    errSum = sum(error);
    errorMean(x) = errSum/x;

       
       plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,estMovement,x,timeToGenerateObject,flowstop,plotTime,collisionTime, timeMovement,error,f_err,errorMean,F_est,F_gt,F_err);
      plotTime(x) = toc;
    end
    
    %%
    %Update position(the objects of interest are tracked via Ground Truth
    %here)

    actualX = xPos(start+iterator);
    actualY = yPos(start+iterator);
    secondActualX = xPos(secondStart+sIterator);
    secondActualY = yPos(secondStart+sIterator);
    
  
    iterator = iterator+1;
    sIterator = sIterator+1;
    
    imwrite(frame,name_frame);
end

%Create Video out of frames.
writer = VideoWriter('Movement.avi');
open(writer);
for i = 1:maxIteration
    name  = sprintf('./Videos/Frames/%06d.png',i);
    img = imread(name);
    writeVideo(writer,img);
end
close(writer);
