%Estimates the Optical Flow, and the collision prediction based on it. 



%FOR LK http://de.mathworks.com/matlabcentral/fileexchange/23142-iterative-pyramidal-lk-optical-flow


clear all;
close all;

load('Initialize.mat');
load('collisionVector.mat');

opticFlow=opticalFlowFarneback;%('NoiseThreshold',0.004);



for x = 1:maxIteration

    if x == 1
    iterator = 0;
    sIterator = 0;
    end
    
    if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end
    if sIterator+secondStart > length(xPos)
        secondStart = 1;
        sIterator = 0;
    end
        
    %Generate Object
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
  
    %create the frame
    disp(x);
    frame = movement(xSpec,ySpec,secondXSpec,secondYSpec);
    
    %%
    %Optical Flow
   frame_gray = rgb2gray(frame);
   flow_frame = estimateFlow(opticFlow,frame_gray);
   
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
        %%collision checkers
       if x > 1

         [estimatedCollision,estMovement] = flowCollision(flow, xSpec,ySpec, secondXSpec,secondYSpec);

        
       if estimatedCollision == 1
            estimatedCollisionVector(x) = 1;
       else
           estimatedCollisionVector(x) = 0;
       end

      plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,estMovement,x);
      end
    
    
    %Update position

    actualX = xPos(start+iterator);
    actualY = yPos(start+iterator);
    secondActualX = xPos(secondStart+sIterator);
    secondActualY = yPos(secondStart+sIterator);
    
  
    iterator = iterator+1;
   sIterator = sIterator+1;
   

    
end