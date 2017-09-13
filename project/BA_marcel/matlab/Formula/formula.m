%Generate movement according to mathematical expressions and not with
%static variables.

%TODO Implement Vikas function
%Graph for GTCOL
%

clear all;

frame = zeros(375,1242,3,'uint8');

%object specs
width = 30;
height = 80;


%starting point
xOrigin = 220;
yOrigin = 70;

secondXOrigin = 470;
secondYOrigin = 45;

actualX = xOrigin;
actualY = yOrigin;

secondActualX = secondXOrigin;
secondActualY = secondYOrigin;

%how many interations?
maxIteration = 200

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.04);
figure(1)

for x = 1:maxIteration
    
    %Position to update frame.Enter formula to calculate xOrigin and
    %yOrign(2nd x- and yOrigin)
    xPos(x) = round(x);
    yPos(x) = round(15*sin(0.5*x));
    
    secondXPos(x) = -round(x);
    secondYPos(x) = round(15*sin(0.5*x));
    
    %%
    %Ground Truth Movement
    [calcMove] = gtMovement(xPos,yPos,secondXPos,secondYPos,x);
    
    %absolute Ground Truth
    absoluteGroundTruth = gT(calcMove,actualX,actualY,secondActualX,secondActualY,width,height,'absolute');
    
    %Generate Object
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
  
    %create the frame
    frame = movement(xSpec,ySpec,secondXSpec,secondYSpec);
    
    %%
    %Optical Flow
    frame_gray = rgb2gray(frame);
    flow_frame = estimateFlow(opticFlow,frame_gray);
    
      %create flow matrix to store the estimated displacemend in.
        flow(:,:,1) = flow_frame.Vx;
        flow(:,:,2) = flow_frame.Vy;
        
        %%
        %%collision checkers
    if x == 1
        [gtCol,iteration] = gtCollision(maxIteration, width,height,actualX,actualY,secondActualX,secondActualY);
    
        
     %check flow collision
     
       if gtCol == 1
            disp('Ground Truth: The objects will collide.');
            plotterColision = 1;
            disp(iteration);
            pause(3);
       end
    end
         [estimatedCollision,estMovement] = flowCollision(flow, xSpec,ySpec, secondXSpec,secondYSpec);

        
        if estimatedCollision == 1
            disp('FLOW: Collision in the future');
        end
%%
            %for better plotting we negate the yMovement
    negatedHeight = ySpec -calcMove(2);
    negatedSecondObjectHeight = secondYSpec -calcMove(4);

    
    if x > 1
    plotter(frame,flow_frame,estMovement,actualX,actualY,secondActualX,secondActualY,gtCol,estimatedCollision,negatedHeight,negatedSecondObjectHeight);
    end
    
    %Update position
    actualX = actualX+calcMove(1);
    actualY = actualY+calcMove(2);
    secondActualX = secondActualX+calcMove(3);
    secondActualY = secondActualY+calcMove(4);
    
end





