function [collision,iteration] = gtCollision(maxIteration, width,height,actualX,actualY,secondActualX,secondActualY)%GTCOLLISION Summary of this function goes here

% Calculate through the iterations and check if the objects are going to
% collide and also check when this will be the case.
% Change formula according to real movement formula. 
% acutal movement.

%TODO SPLIT PROCESS.
collision = 0;
iteration = 0;

 for x=1:maxIteration
%Used formula in main programm
 xPos(x) = round(x);
 yPos(x) = round(5*sin(0.5*x));
    
 secondXPos(x) = -round(x);
 secondYPos(x) = round(5*sin(0.5*x));
 
 %get movement for eacht iteration
 [calcMove] = gtMovement(xPos,yPos,secondXPos,secondYPos,x);
 
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
    actualX = actualX+calcMove(1);
    actualY = actualY+calcMove(2);
    secondActualX = secondActualX+calcMove(3);
    secondActualY = secondActualY+calcMove(4);
    

%check if an unavoidable collision will occur each round
    xCol = xSpec+calcMove(1);
    yCol = ySpec+calcMove(2);
    secondXCol = secondXSpec+calcMove(3);
    secondYCol = secondYSpec+calcMove(4);
    
    checkX = [];
    checkY = [];
    checkX = intersect(xCol,secondXCol);
    checkY = intersect(yCol,secondYCol);
    
    if ~isempty(checkX) & ~isempty(checkY)
        collision = 1;
        iteration = x;
        break;
    end
    
end
end

   





