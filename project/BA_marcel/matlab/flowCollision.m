function [ estCollision,movement ] = flowCollision(absoluteFlow,flow, width,height, secondObjectWidth,secondObjectHeight)

%% Estimate if the objects will collide
% We extract the movement from the objects by using the mean of the flow
% components. We then add the movement to the actual position and get the
% future position. After 10 iteration we will say if the objects will
% collide or not. This will be done with each consequtive frame pair. 
%
estCollision = 0;

%Threshold of the object, because object detection is not accurate.
if height < 365
upperheigtht = height(end)+10;
end
if height > 10
    lowerheight = height(1)-10;
end

if width < 1232
upperwidth = width(end)+10;
end
if width > 10
lowerwidth = width(1)-10;
end


if secondObjectHeight < 365
secondObjectUpperheigtht = secondObjectHeight(end)+10;
end
if secondObjectHeight > 10
    secondObjectLowerheigtht = secondObjectHeight(1)-10;
end

if secondObjectWidth < 1232
secondObjectUpperWidth = secondObjectWidth(end)+10;
end
if secondObjectWidth > 10
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

disp('Estimated Movement of the First Object:');
disp('x')
disp(xMean);
disp('y')
disp(yMean);
disp('Estimated Movement of the second Object');
disp('x')
disp(secondObjectXMean);
disp('y')
disp(secondObjectYMean);

movement = [xMean,yMean,secondObjectXMean,secondObjectYMean];

%%
%Estimate the future collision. Floor call in order to get possibly matching results
for i=1:10
    
    height = floor( height+yMean);
    width = floor(width+xMean);
    secondObjectHeight = floor(secondObjectHeight+secondObjectYMean);
    secondObjectWidth = floor(secondObjectWidth+secondObjectXMean);
    
    checkY = intersect(height,secondObjectHeight);
    checkX = intersect(width,secondObjectWidth);
    
    if ~isempty(checkX) & ~isempty(checkY)
        estCollision = 1;
        break;
    end
    
end





