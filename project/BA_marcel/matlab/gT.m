% This function creates ground truth data in two formats:
% The groundTruthRelativeMatrix and groundTruthAbsoluteMatrix - relative 
% and absolute matrix. 


function [responseMatrix] = gT( height,width,xMovement,yMovement,...
secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement, parameter )

%%generate Ground Truth
groundTruthRelativeMatrix = zeros(375,1242,3,'uint16');
groundTruthAbsoluteMatrix = zeros(375,1242,3);
responseMatrix = zeros(375,1242,3);

for k=height
    for j=width
        groundTruthRelativeMatrix(k,j,1) = xMovement;
        groundTruthRelativeMatrix(k,j,2) = yMovement;
        groundTruthRelativeMatrix(k,j,3) = 1;
        groundTruthAbsoluteMatrix(k,j,1) = xMovement+j;
        groundTruthAbsoluteMatrix(k,j,2) = yMovement+k;
        groundTruthAbsoluteMatrix(k,j,3) = 1;
    end
end

for k=secondObjectHeight
    for j=secondObjectWidth
        groundTruthRelativeMatrix(k,j,1) = secondXMovement;
        groundTruthRelativeMatrix(k,j,2) = secondYMovement;
        groundTruthRelativeMatrix(k,j,3) = 1;
        groundTruthAbsoluteMatrix(k,j,1) = secondXMovement+j;
        groundTruthAbsoluteMatrix(k,j,2) = secondYMovement+k;
        groundTruthAbsoluteMatrix(k,j,3) = 1;
    end
end

%%Quantization
gtX = (groundTruthRelativeMatrix(:,:,1)*64)+2^15;
gtY = (groundTruthRelativeMatrix(:,:,2)*64)+2^15;

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
kittiColorCodedGroundTruth = cat(3,groundTruthRelativeMatrix(:,:,1),...
      groundTruthRelativeMatrix(:,:,2),groundTruthRelativeMatrix(:,:,3));
kittiColorCodedGroundTruth =  uint16(kittiColorCodedGroundTruth);

if parameter == 'relative'
    responseMatrix = groundTruthRelativeMatrix
elseif parameter == 'absolute'
    responseMatrix = groundTruthAbsoluteMatrix
elseif parameter == 'kitti'
    responseMatrix = kittiColorCodedGroundTruth
end

end

