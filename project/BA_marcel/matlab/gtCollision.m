function [collision] = gtCollision(maxIteration, absoluteGT, width,height,xMovement,yMovement, secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement,counter )

% use values from absoluteGT to calculate if a collision appears with the
% acutal movement. 

%check if an unavoidable collision will occur each round
collision = 0;


%estimate future collisions based on actual movement 
%therefore move the object virtually into the direction.

if counter == 0

for i=1:maxIteration

    height = height+yMovement;
    width = width+xMovement;
    secondObjectHeight = secondObjectHeight+secondYMovement;
    secondObjectWidth = secondObjectWidth+secondXMovement;
    
    checkY = intersect(height,secondObjectHeight);
    checkX = intersect(width,secondObjectWidth);
    
    if ~isempty(checkX) & ~isempty(checkY)
        collision = 1;
        break;
    end
    
end
end
end
   
    



