function [ movement ] =  gtMovement(xPos,yPos,start,sStart,iterator,sIterator,x)



if x == 1
    xMovement = 0;
    secondXMovement = 0;
    yMovement = 0;
    secondYMovement = 0;
    movement = [xMovement,yMovement,secondXMovement,secondYMovement];
    return;
end

if iterator+start > length(xPos)
    
    xMovement = xPos(1) - xPos(length(xPos));
    yMovement = yPos(1) - yPos(length(yPos));
end

    
    
if sIterator+sStart > length(xPos)
    secondXMovement = xPos(1) - xPos(length(xPos));
    secondYMovement = yPos(1) - yPos(length(yPos));
end

    
    
    
if iterator+start <= length(xPos)
    xMovement = xPos(start+iterator) - xPos(start+iterator-1);
    yMovement = yPos(start+iterator) - yPos(start+iterator-1);
end
    

if sIterator+sStart <= length(xPos)
    secondXMovement = xPos(sStart+sIterator) - xPos(sStart+sIterator-1);
        secondYMovement = yPos(sStart+sIterator) - yPos(sStart+sIterator-1);
end





movement = [xMovement,yMovement,secondXMovement,secondYMovement];
end



    
    




