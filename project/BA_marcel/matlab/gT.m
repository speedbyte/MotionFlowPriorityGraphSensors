function [response] = gT( height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement,parameter )
%%generate GT

relativeGroundTruth = zeros(375,1242,3,'uint16');
absoluteGroundTruth = zeros(375,1242,3);

for k=height
    for j=width
        relativeGroundTruth(k,j,1) = xMovement;
        relativeGroundTruth(k,j,2) = yMovement;
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = xMovement+j;
        absoluteGroundTruth(k,j,2) = yMovement+k;
        absoluteGroundTruth(k,j,3) = 1;
    end
end

for k=secondObjectHeight
    for j=secondObjectWidth
        relativeGroundTruth(k,j,1) = secondXMovement;
        relativeGroundTruth(k,j,2) = secondYMovement;
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = secondXMovement+j;
        absoluteGroundTruth(k,j,2) = secondYMovement+k;
        absoluteGroundTruth(k,j,3) = 1;
    end
end

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));
kittiGT =  uint16(kittiGT);

if strcmp(parameter,'absolute')
    response = absoluteGroundTruth;
elseif strcmp(parameter,'relative')
    response = relativeGroundTruth;
elseif strcmp(parameter,'kitti')
    response = kittiGT;
end


end

