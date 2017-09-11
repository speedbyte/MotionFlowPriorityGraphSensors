%Generate movement according to mathematical expressions and not with
%static variables. 

%ToDo Code Refactoring
% GT collision
% flow collision
%plotter


clear all;
frame = zeros(375,1242,3,'uint8');

%object specs
width = 30;
height = 80;


%starting point
xOrigin = 220;
yOrigin = 70;

secondXOrigin = 500;
secondYOrigin = 45;

%how many interations?
it = 100



opticFlow=opticalFlowFarneback%('NoiseThreshold',0.04);
figure(1)
for x = 1:it
    
    %Position to update frame
    xPos(x) = round(x);
    yPos(x) = round(5*sin(0.5*x));
    
    secondXPos(x) = -round(x);
    secondYPos(x) = round(5*sin(0.5*x));
    
    %%
    %Ground Truth Movement
    if length(xPos) == 1
        xMovement = xPos(x);
    else
        xMovement = xPos(x) - xPos(x-1);
    end
    
    if length(yPos) == 1
        yMovement = yPos(x);
    else
        yMovement = yPos(x) - yPos(x-1);
    end
    
    
     if length(secondXPos) == 1
        secondXMovement = secondXPos(x);
    else
        secondXMovement = secondXPos(x) - secondXPos(x-1);
    end
    
    if length(secondYPos) == 1
        secondYMovement = secondYPos(x);
    else
        secondYMovement = secondYPos(x) - secondYPos(x-1);
    end
    
    
    xSpec = xOrigin+xPos(x):xOrigin+xPos(x)+width;  %width
    ySpec = yOrigin+yPos(x):yOrigin+yPos(x)+height; %height
    secondXSpec = secondXOrigin+secondXPos(x):secondXOrigin+secondXPos(x)+width;
    secondYSpec = secondYOrigin+secondYPos(x):secondYOrigin+secondYPos(x)+height;

    %%
    %reset the image to white
    for k=1:375
        for j=1:1242
            for i=1:3
                frame(k,j,i) = 255;
            end
        end
    end
    %draw new image.
    for k = ySpec
        for j= xSpec
            for i=1:3
                frame(k,j,i)= 0;
            end
        end
    end
    
    
    %expand with 2nd Object
    for k = secondYSpec
        for j= secondXSpec
            for i=1:3
                frame(k,j,i)= 0;
            end
        end
    end
    
    %%
    %Optical Flow
   frame_gray = rgb2gray(frame);
   flow_frame = estimateFlow(opticFlow,frame_gray);
   
   imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[10 10],'ScaleFactor',6);
drawnow;
title('Optical FLow');
hold off;
    
    
end




