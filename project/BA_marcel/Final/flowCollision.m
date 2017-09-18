function [ estCollision ] = flowCollision(absoluteFlow, xSpec,ySpec, secondXSpec,secondYSpec)

%% Estimate if the objects will collide
% Substract the absolute Flows of the both objects of interest. If the
% result is <= width and height of the object, the objects are colliding
%
estCollision = 0;


%%
%Estimate the future collision. Floor call in order to get possibly matching results
    
for k = 1:length(ySpec)
    for j = 1:length(xSpec)
        flow1(k,j,1) = absoluteFlow(ySpec(k),xSpec(j),1);
        flow1(k,j,2) = absoluteFlow(ySpec(k),xSpec(j),2);
    end
end

for k = 1:length(secondYSpec)
    for j = 1:length(secondXSpec)
        flow2(k,j,1) = absoluteFlow(secondYSpec(k),secondXSpec(j),1);
        flow2(k,j,2) = absoluteFlow(secondYSpec(k),secondXSpec(j),2);
    end
end
abs1 = absoluteFlow(:,:,1);
f1 = flow1(:,:,1);
f2 = flow2(:,:,1);
f3 = f1-f2;
finalFlow = flow1-flow2;
finalFlow = abs(finalFlow);


sizeCheckX = length(xSpec);
sizeCheckY = length(ySpec);

xChecker = ismember([0:sizeCheckX],finalFlow(:,:,1));
yChecker = ismember([0:sizeCheckY],finalFlow(:,:,2));

if ismember(1,xChecker) & ismember(1,yChecker)
    estCollision = 1;
    end

    
end

    
