function [ output_args ] = gtCollision( name,folder,vName )

load(name);

tic;


%Generates the ground truth Kitti Maps and the Ground Truth collision



collision = 0;
for x=1:maxIteration
    
    %Used to store the GT images for the kitti devkit
  name_GT = sprintf('./../../../../matlab_dataset/data/stereo_flow/flow_occ_%s/%06d_10.png',folder,x);

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
        
       relativeGroundTruth = zeros(375,1242,3,'single');
       absoluteGroundTruth = zeros(375,1242,3,'uint16');

 %%  
   %calculating the relative Ground Truth for the Kitti devkit and store it
   %in a png file
   for k = ySpec
    for j= xSpec
        relativeGroundTruth(k,j,1) = calcMove(1);
        relativeGroundTruth(k,j,2) = calcMove(2);
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = calcMove(1)+j;
        absoluteGroundTruth(k,j,2) = calcMove(2)+k;
        absoluteGroundTruth(k,j,3) = 1;

    end
   end

for k=secondYSpec
    for j=secondXSpec
        relativeGroundTruth(k,j,1) = calcMove(3);
        relativeGroundTruth(k,j,2) = calcMove(4);
        relativeGroundTruth(k,j,3) = 1;
        absoluteGroundTruth(k,j,1) = calcMove(3)+j;
        absoluteGroundTruth(k,j,2) = calcMove(4)+k;
        absoluteGroundTruth(k,j,3) = 1;

    end
end

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));

     addpath(genpath('../../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
     flow_write(kittiGT,name_GT);
   

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

disp(x);

end





%store the Ground Truth collision vector
if vName == 0
fileName = 'collisionVectorSlow.mat';
end

if vName == 1
fileName = 'collisionVectorNormal.mat';
end

if vName == 2
    fileName = 'collisionVectorFast.mat';
end

save(fileName,'collisionVector');
time = toc;
disp(time);



end

