function [ response ] = gT( movement,actualX,actualY,secondActualX,secondActualY,width,height,parameter )
%Calculates x and y Movement of the two objects.
%TODO IMWRITE KITTI

relativeGroundTruth = zeros(375,1242,3,'int16');
absoluteGroundTruth = zeros(375,1242,3);


for k = actualY:actualY+height
    for j= actualX:actualX+width
        relativeGroundTruth(k,j,1) = movement(1);
        relativeGroundTruth(k,j,2) = movement(2);
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = movement(1)+j;
        absoluteGroundTruth(k,j,2) = movement(2)+k;
        absoluteGroundTruth(k,j,3) = 1;
        
    end
end

for k=secondActualY:secondActualY+height
    for j=secondActualX:secondActualX+width
        relativeGroundTruth(k,j,1) = movement(3);
        relativeGroundTruth(k,j,2) = movement(4);
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = movement(3)+j;
        absoluteGroundTruth(k,j,2) = movement(4)+k;
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

