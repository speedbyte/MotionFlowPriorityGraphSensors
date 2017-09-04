function [collision] = gtCollision( absoluteGT, width,height,xMovement,yMovement, secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement )

% use values from absoluteGT to calculate if a collision appears with the
% acutal movement. 

%check if an unavoidable collision will occur each round
collision = 0;

for k = height
    for j = width
        for kk = secondObjectHeight
            for jj = secondObjectWidth
                if (absoluteGT(k,j,1) == absoluteGT(kk,jj,1)) && (absoluteGT(k,j,2) == absoluteGT(kk,jj,2))
                    collision = 1;
                end 
            end 
        end
    end 
end

if collision == 1
    disp('Unavoidable collision detected via Ground Truth');
end

%TODO estimate future collisions based on actual movement 

