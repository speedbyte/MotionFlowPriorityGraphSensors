function [iteration] = gtCollision()%GTCOLLISION Summary of this function goes here

% Calculate through the iterations and check if the objects are going to
% collide and also check when this will be the case.
% Change formula according to real movement formula. 
% acutal movement.

iteration = 0;
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


%TODO SPLIT PROCESS.
collision = 0;


 for x=1:1000

    if x == 1
    iterator = 0;
    sIterator = 0;
    end
   
    
    
    %%
    %Ground Truth Movement
    [calcMove] = gtMovement(xPos,yPos,start,sStart,iterator,sIterator,x);
    
     if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end

    if sIterator+sStart > length(xPos)
        sStart = 1;
        sIterator = 0;
    end
    
  
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
    actualX = actualX+calcMove(1);
    actualY = actualY+calcMove(2);
    secondActualX = secondActualX+calcMove(3);
    secondActualY = secondActualY+calcMove(4);
    

%check if an unavoidable collision will occur next round
    xCol = xSpec+calcMove(1);
    yCol = ySpec+calcMove(2);
    secondXCol = secondXSpec+calcMove(3);
    secondYCol = secondYSpec+calcMove(4);
    

    checkX = intersect(xCol,secondXCol);
    checkY = intersect(yCol,secondYCol);
    
    if ~isempty(checkX) & ~isempty(checkY)
        collision = collision+1;
        iteration(collision) = x;
    end
    iterator = iterator+1;
   sIterator = sIterator+1;


    
end
end

   





