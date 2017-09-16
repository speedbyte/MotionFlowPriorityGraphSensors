function [ estCollision ] = flowCollision(absoluteFlow, xSpec,ySpec, secondXSpec,secondYSpec)

%% Estimate if the objects will collide
% We extract the movement from the objects by using the mean of the flow
% components. We then add the movement to the actual position and get the
% future position. Then we will say if the objects will
% collide or not. This will be done with each consequtive frame pair. 
%
estCollision = 0;

flow1 = ones(375,1242,'int16');
flow2 = zeros(375,1242,'int16');


%%
%Estimate the future collision. Floor call in order to get possibly matching results
    
for k = ySpec
    for j = xSpec
        flow1(k,j,1) = absoluteFlow(k,j,1);
        flow1(k,j,2) = absoluteFlow(k,j,2);
    end
end

for k = secondYSpec
    for j = secondXSpec
        flow2(k,j,1) = absoluteFlow(k,j,1);
        flow2(k,j,2) = absoluteFlow(k,j,2);
    end
end
finalFlow = flow1-flow2;

if ismember(0,finalFlow(:,:,1)) & ismember(0,finalFlow(:,:,2))
    estCollision = 1;
    end

    
end

    
