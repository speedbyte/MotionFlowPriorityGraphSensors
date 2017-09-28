tic;
load('Initialize.mat');


%Generates the ground truth Kitti Maps and the Ground Truth collision



collision = 0;
for x=1:maxIteration
    
    %Used to store the GT images for the kitti devkit
  name_dense = sprintf('./GroundTruth/%06d_10.png',x);

    %Initialization
    if x == 1
    iterator = 0;
    sIterator = 0;
    end
    
    %%
    %Ground Truth Movement (First Object x, first Object y, second object
    %x, second object y movement
    [calcMove] = gtMovement(xPos,yPos,start,secondStart,iterator,sIterator,x);
    
    %If we are at the end of the path vector, we need to reset our
    %iterators
     if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end

    if sIterator+secondStart > length(xPos)
        secondStart = 1;
        sIterator = 0;
    end
    
  
    %Object specification
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
       relativeGroundTruth = zeros(375,1242,3,'uint16');
       absoluteGroundTruth = zeros(375,1242,3,'uint16');

 %%  
   %calculating the relative Ground Truth for the Kitti devkit and store it
   %in a png file
   for k = ySpec
    for j= xSpec
        relativeGroundTruth(k,j,1) = calcMove(1)*64+2^15;
        relativeGroundTruth(k,j,2) = calcMove(2)*64+2^15;
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = calcMove(1)+j;
        absoluteGroundTruth(k,j,2) = calcMove(2)+k;
        absoluteGroundTruth(k,j,3) = 1;

    end
   end

for k=secondYSpec
    for j=secondXSpec
        relativeGroundTruth(k,j,1) = calcMove(3)*64+2^15;
        relativeGroundTruth(k,j,2) = calcMove(4)*64+2^15;
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = calcMove(3)+j;
        absoluteGroundTruth(k,j,2) = calcMove(4)+k;
        absoluteGroundTruth(k,j,3) = 1;

    end
end

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));
kittiGT =  uint16(kittiGT);

imwrite(kittiGT,name_dense);
   

%%    

%check for each frame (iteration) if the objects are colliding
    xCol = xSpec;
    yCol = ySpec;
    secondXCol = secondXSpec;
    secondYCol = secondYSpec;
    
    checkX = intersect(xCol,secondXCol);
    checkY = intersect(yCol,secondYCol);
    
    if ~isempty(checkX) & ~isempty(checkY)
        collisionVector(x) = 1;
    else
        collisionVector(x) = 0;
    end
    
    iterator = iterator+1;
   sIterator = sIterator+1;
       
    actualX = actualX+calcMove(1);
    actualY = actualY+calcMove(2);
    secondActualX = secondActualX+calcMove(3);
    secondActualY = secondActualY+calcMove(4);



end





%store the Ground Truth collision vector
fileName = 'collisionVector';
save(fileName,'collisionVector');
time = toc;
disp(time);