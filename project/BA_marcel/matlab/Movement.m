function [ img1 ] = Movement( height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement )
%%Helper function. Responsible for the movement of the objects


%white background
for k=1:375
    for j=1:1242
        for i=1:3
            img1(k,j,i) = 255;            
        end
    end
end

 
%%create shifted frame
for k = height
    for j= width
        for i=1:3
            img1(k+yMovement,j+xMovement,i)= 0;

        end
    end
end


%%expand with shifted 2nd object
for k = secondObjectHeight
    for j= secondObjectWidth
        for i=1:3
            img1(k,j,i)= 0;

        end
    end
end
end

