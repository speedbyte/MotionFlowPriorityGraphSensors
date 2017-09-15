%Loads the Initialization specs. 

%video
%kitti einbinden
%Document 
%keep matrix size
%only use absolut coordinates
%make everything more generic
%classify everything that moves as object
%noise(static, dynamic)



%calibration file, 2 persons have checkerboard. Fotos to define start
%positions.


%FOR LK http://de.mathworks.com/matlabcentral/fileexchange/23142-iterative-pyramidal-lk-optical-flow


clear all;
close all;

load('Initialize.mat');
load('collisionVector.mat');

opticFlow=opticalFlowFarneback;%('NoiseThreshold',0.004);
frame = zeros(375,1242,3,'uint8');
plotTime = 1;


for x = 1:maxIteration

    %Initialization
    if x == 1
    iterator = 0;
    sIterator = 0;
    end
    
    lastFrame = frame;
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
    
  
    %create the frame
    disp(x);
    frame = movement(xSpec,ySpec,secondXSpec,secondYSpec);
    timeToGenerateObject(x) = toc;
    
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
        
        %%
        %%collision checkers(first round has bad flow, dont plot and
        %%estimate collision
        %time to check for collision
        tic;

         [estimatedCollision,estMovement] = flowCollision(flow, xSpec,ySpec, secondXSpec,secondYSpec);

        
       if estimatedCollision == 1
            estimatedCollisionVector(x) = 1;
       else
           estimatedCollisionVector(x) = 0;
       end
       timeCollision(x) = toc;
        
       tic;
      plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,estMovement,x,timeToGenerateObject,flowstop,timeCollision,plotTime);
      plotTime(x) = toc;
    
    %%
    %Update position

    actualX = xPos(start+iterator);
    actualY = yPos(start+iterator);
    secondActualX = xPos(secondStart+sIterator);
    secondActualY = yPos(secondStart+sIterator);
    
  
    iterator = iterator+1;
    sIterator = sIterator+1;

    
end