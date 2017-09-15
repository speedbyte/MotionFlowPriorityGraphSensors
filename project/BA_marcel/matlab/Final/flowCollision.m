function [ estCollision,movement ] = flowCollision(flow, xSpec,ySpec, secondXSpec,secondYSpec)

%% Estimate if the objects will collide
% We extract the movement from the objects by using the mean of the flow
% components. We then add the movement to the actual position and get the
% future position. After 10 iteration we will say if the objects will
% collide or not. This will be done with each consequtive frame pair. 
%
estCollision = 0;
flowFirstObjectX = zeros(375,1242);
flowFirstObjectY = zeros(375,1242);
flowSecondObjectX = zeros(375,1242);
flowSecondObjectY = zeros(375,1242);

%Threshold of the object, because object detection is not accurate.
upperheigtht = ySpec(end);
lowerheight = ySpec(1);
upperwidth = xSpec(end);
lowerwidth = xSpec(1);
secondObjectUpperheigtht = secondYSpec(end);
secondObjectLowerheigtht = secondYSpec(1);
secondObjectUpperWidth = secondXSpec(end);
secondObjectLowerWidth = secondObjectUpperWidth(1)-10


if ySpec < 365
upperheigtht = ySpec(end)+10;
end
if ySpec > 10
    lowerheight = ySpec(1)-10;
end

if xSpec < 1232
upperwidth = xSpec(end)+10;
end
if xSpec > 10
lowerwidth = xSpec(1)-10;
end


if secondYSpec < 365
secondObjectUpperheigtht = secondYSpec(end)+10;
end
if secondYSpec > 10
    secondObjectLowerheigtht = secondYSpec(1)-10;
end

if secondXSpec < 1232
secondObjectUpperWidth = secondXSpec(end)+10;
end
if secondXSpec > 10
secondObjectLowerWidth = secondObjectUpperWidth(1)-10;
end
%%
%%get flow from the objects
for k = lowerheight:upperheigtht
    for j = lowerwidth:upperwidth
        flowFirstObjectX(j,k,1) = flow(k,j,1);
        flowFirstObjectY(j,k,2) = flow(k,j,2);
    end
end

for kk = secondObjectLowerheigtht:secondObjectUpperheigtht
    for jj = secondObjectLowerWidth:secondObjectUpperWidth
        flowSecondObjectX(jj,kk,1) = flow(kk,jj,1);
        flowSecondObjectY(jj,kk,2) = flow(kk,jj,2);
    end
end

%%
%Extract the movement of the object.
firstObjectX = nonzeros(flowFirstObjectX);
xThreshold = find(abs(firstObjectX)<0.2); %cutting out very small displacements
firstObjectX(xThreshold) = [];

firstObjectY= nonzeros(flowFirstObjectY);
yThreshold = find(abs(firstObjectY)<0.2);
firstObjectY(yThreshold) = [];

secondObjectX = nonzeros(flowSecondObjectX);
xThreshold = find(abs(secondObjectX)<0.2); 
secondObjectX(xThreshold) = [];

secondObjectY = nonzeros(flowSecondObjectY);
yThreshold = find(abs(secondObjectY)<0.2);
secondObjectY(yThreshold) = [];
%%

%%
%Get the movement by getting the mean of the flow objects. 
xMean=mean(firstObjectX);
yMean=mean(firstObjectY);
secondObjectXMean = mean(secondObjectX);
secondObjectYMean = mean(secondObjectY);

if isnan(xMean)
    xMean = 0;
end
if isnan(yMean)
    yMean = 0;
end
if isnan(secondObjectYMean)
    secondObjectYMean = 0;
end
if isnan(secondObjectXMean)
    secondObjectXMean = 0;
end
% 
% disp('Estimated Movement of the First Object:');
% disp('x')
% disp(xMean);
% disp('y')
% disp(yMean);
% disp('Estimated Movement of the second Object');
% disp('x')
% disp(secondObjectXMean);
% disp('y')
% disp(secondObjectYMean);

movement = [xMean,yMean,secondObjectXMean,secondObjectYMean];

%%
%Estimate the future collision. Floor call in order to get possibly matching results
    
    ySpec = floor(ySpec+yMean);
    xSpec = floor(xSpec+xMean);
    secondYSpec = floor(secondYSpec+secondObjectYMean);
    secondXSpec = floor(secondXSpec+secondObjectXMean);
    
    checkY = intersect(ySpec,secondYSpec);
    checkX = intersect(xSpec,secondXSpec);
    
    if ~isempty(checkX) & ~isempty(checkY)
        estCollision = 1;
        break;
    end
    
