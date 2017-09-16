function [ movement ] = estimatedMovement( flow, xSpec,ySpec, secondXSpec,secondYSpec )

%Threshold of the object, because object detection is not accurate.
upperheigtht = ySpec(end);
lowerheight = ySpec(1);
upperwidth = xSpec(end);
lowerwidth = xSpec(1);

secondObjectUpperheight = secondYSpec(end);
secondObjectLowerheight = secondYSpec(1);
secondObjectUpperWidth = secondXSpec(end);
secondObjectLowerWidth = secondXSpec(1);

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
secondObjectUpperheight = secondYSpec(end)+10;
end
if secondYSpec > 10
    secondObjectLowerheight = secondYSpec(1)-10;
end

if secondXSpec < 1232
secondObjectUpperWidth = secondXSpec(end)+10;
end
if secondXSpec > 10
secondObjectLowerWidth = secondXSpec(1)-10;
end

%%
%%get flow from the objects
for k = lowerheight:upperheigtht
    for j = lowerwidth:upperwidth
        flowFirstObjectX(j,k,1) = flow(k,j,1);
        flowFirstObjectY(j,k,2) = flow(k,j,2);
    end
end

for kk = secondObjectLowerheight:secondObjectUpperheight
    for jj = secondObjectLowerWidth:secondObjectUpperWidth
        flowSecondObjectX(jj,kk,1) = flow(kk,jj,1);
        flowSecondObjectY(jj,kk,2) = flow(kk,jj,2);
    end
end
%%
%Extract the movement of the object.
tic;
firstObjectX = nonzeros(flowFirstObjectX);
% xThreshold = find(abs(firstObjectX)<0.2); %cutting out very small displacements
% firstObjectX(xThreshold) = [];
test = toc

firstObjectY= nonzeros(flowFirstObjectY);
% yThreshold = find(abs(firstObjectY)<0.2);
% firstObjectY(yThreshold) = [];

secondObjectX = nonzeros(flowSecondObjectX);
% xThreshold = find(abs(secondObjectX)<0.2); 
% secondObjectX(xThreshold) = [];

secondObjectY = nonzeros(flowSecondObjectY);
% yThreshold = find(abs(secondObjectY)<0.2);
% secondObjectY(yThreshold) = [];
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


end

