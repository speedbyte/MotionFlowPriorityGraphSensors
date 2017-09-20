%Generate movement according to mathematical expressions and not with
%static variables.

%TODO Implement Vikas function
%Graph for GTCOL
%

clear all;
close all;

theta = 1:0.5:360;
    for x=1:719
    xPos(x)=600+round(500*cos(theta(x)*3.14/180)/(1+power(sin(theta(x)*3.14/180),2)));
    yPos(x)= 150+round(100*(cos(theta(x)*3.14/180)*sin(theta(x)*3.14/180))/(0.2+power(sin(theta(x)*3.14/180),2)));
    end
%object specs
width = 30;
height = 80;

  start = 718;
    
 %Start is somewhere on the path
 xOrigin = xPos(start);
 yOrigin = yPos(start);
 
 sStart = 718;
 secondXOrigin = xPos(sStart);
 secondYOrigin = yPos(sStart);
 
actualX = xOrigin;
actualY = yOrigin;

secondActualX = secondXOrigin;
secondActualY = secondYOrigin;


%how many interations?
maxIteration = 1000;

for x = 1:maxIteration

    if x == 1
    iterator = 0;
    sIterator = 0;
    end

    
    
    
    %%
    %Ground Truth Movement
    [calcMove] =  gtMovement(xPos,yPos,start,sStart,iterator,sIterator,x);
    
    if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end
    if sIterator+sStart > length(xPos)
        sStart = 1;
        sIterator = 0;
    end
    
    %absolute Ground Truth
    absoluteGroundTruth = gT(calcMove,actualX,actualY,secondActualX,secondActualY,width,height,'absolute');
    
    %Generate Object
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
  
    %create the frame
    disp(x)
    frame = movement(xSpec,ySpec,secondXSpec,secondYSpec);
    
    %%
    %Optical Flow
%   frame_gray = rgb2gray(frame);
%   flow_frame = estimateFlow(opticFlow,frame_gray);
    
      %create flow matrix to store the estimated displacemend in.
%        flow(:,:,1) = flow_frame.Vx;
%        flow(:,:,2) = flow_frame.Vy;
        
        %%
        %%collision checkers
    if x == 1
        iteration = gtCollision();
    
        
     %check flow collision
     
       
            plotterColision = 1;
            iteration;
            pause(3);
       
    end
%         [estimatedCollision,estMovement] = flowCollision(flow, xSpec,ySpec, secondXSpec,secondYSpec);

        
%        if estimatedCollision == 1
%            disp('FLOW: Collision in the future');
%        end
%%
            %for better plotting we negate the yMovement
%    negatedHeight = ySpec -calcMove(2);
%    negatedSecondObjectHeight = secondYSpec -calcMove(4);

    
%    if x > 1
%    plotter(frame,flow_frame,estMovement,actualX,actualY,secondActualX,secondActualY,gtCol,estimatedCollision,negatedHeight,negatedSecondObjectHeight);
%    end
    
    %Update position

    actualX = xPos(start+iterator);
    actualY = yPos(start+iterator);
    secondActualX = xPos(sStart+sIterator);
    secondActualY = yPos(sStart+sIterator);
    

imshow(frame);    
%hold on
%plot(flow_frame,'DecimationFactor',[5 5],'ScaleFactor',10);
%drawnow;
%title('Optical FLow');
%hold off;;
    
    iterator = iterator+1;
   sIterator = sIterator+1;
   if x == 128
       disp('..');
   end
    
end





