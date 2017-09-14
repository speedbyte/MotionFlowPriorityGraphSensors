load('Initialize.mat');


%Generates the ground truth Kitti Maps and the Ground Truth collision
%prediction.


collision = 0;
for x=1:maxIteration
    
  name_dense = sprintf('./GroundTruth/%06d_10.png',x);

    if x == 1
    iterator = 0;
    sIterator = 0;
    end
   
    
    
    %%
    %Ground Truth Movement
    [calcMove] = gtMovement(xPos,yPos,start,secondStart,iterator,sIterator,x);
    
     if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end

    if sIterator+secondStart > length(xPos)
        secondStart = 1;
        sIterator = 0;
    end
    
  
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
       relativeGroundTruth = zeros(375,1242,3,'int16');
   
   %relative GT
   for k = ySpec
    for j= xSpec
        relativeGroundTruth(k,j,1) = calcMove(1);
        relativeGroundTruth(k,j,2) = calcMove(2);
        relativeGroundTruth(k,j,3) = 1;
    end
end

for k=secondYSpec
    for j=secondXSpec
        relativeGroundTruth(k,j,1) = calcMove(3);
        relativeGroundTruth(k,j,2) = calcMove(4);
        relativeGroundTruth(k,j,3) = 1;
    end
end

rel1 = relativeGroundTruth(:,:,1);
rel2 = relativeGroundTruth(:,:,2);

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));
kittiGT =  uint16(kittiGT);

imwrite(kittiGT,name_dense);
    
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
        collisionVector(x) = 1;
    else
        collisionVector(x) = 0;
    end
    
    iterator = iterator+1;
   sIterator = sIterator+1;
   

    
end

relativeGroundTruth = zeros(375,1242,3,'int16');




fileName = 'collisionVector';
save(fileName,'collisionVector');