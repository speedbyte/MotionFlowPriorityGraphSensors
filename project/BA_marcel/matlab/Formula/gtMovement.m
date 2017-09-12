function [ movement ] = gtMovement( xPos,yPos,secondXPos,secondYPos,x )

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
    
    movement = [xMovement,yMovement,secondXMovement,secondYMovement];
    


end

