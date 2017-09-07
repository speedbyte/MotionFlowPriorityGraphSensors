function [ estCollision ] = flowCollision( absoluteFlow,flow, width,height, secondObjectWidth,secondObjectHeight)

%% (use width+height+threshold) and secondWidth/height/threshold
% work with those values and check for collision then 
% At that moment only say something about unavoidable collisions. 

%TODO Threshold

collision = 0;
estCollision = 0;

for k = height
    for j = width
        for kk = secondObjectHeight
            for jj = secondObjectWidth
                if (absoluteFlow(k,j,1) == absoluteFlow(kk,jj,1)) && (absoluteFlow(k,j,2) == absoluteFlow(kk,jj,2))
                    collision = 1;
                end 
            end 
        end
    end 
end

if collision == 1
    disp('Unavoidable collision via FLOW');
    return;
end
%%
%%get flow from the objects

for k = height
    for j = width
        flowFirstObjectX(:,:,1) = flow(k,j,1);
        flowFirstObjectY(:,:,2) = flow(k,j,2);
    end
end

for kk = secondObjectHeight
            for jj = secondObjectWidth
                flowSecondObjectX(:,:,1) = flow(kk,jj,1);
                flowSecondObjectY(:,:,2) = flow(kk,jj,2);
            end
end

%%
%Extract the movement of the object. 
 x = nonzeros(flowFirstObjectX);
        xThreshold = find(abs(x)<0.09); %cutting out very small displacements
        x(xThreshold) = [];
        
        y = nonzeros(flowFirstObjectY);
        yThreshold = find(abs(y)<0.09);
        y(yThreshold) = [];
        
        xS = nonzeros(flowSecondObjectX);
        xThreshold = find(abs(xS)<0.09); %cutting out very small displacements
        xS(xThreshold) = [];
        
        yS = nonzeros(flowSecondObjectY);
        yThreshold = find(abs(yS)<0.09);
        y(yThreshold) = [];
        
        
        
        xMean=mean(x);
        yMean=mean(y);
        xSMean = mean(xS);
        ySMean = mean(yS);
        
        disp('Estimated Movement of the First Object:');
        
        disp('x')
        disp(xMean);
        disp('y')
        disp(yMean);
        disp('Estimated Movement of the second Object');
        disp('x')
        disp(xSMean);
        disp('y')
        disp(ySMean);
        
        if isnan(xMean)
            xMean = 0;
        end
        if isnan(yMean)
            yMean = 0;
        end
        if isnan(ySMean)
            ySMean = 0;
        end
        if isnan(xSMean)
            xSMean = 0;
        end
%%
%Estimate the future collision

for i=1:10

    height = floor( height+yMean);
    width = floor(width+xMean);
    secondObjectHeight = floor(secondObjectHeight+ySMean);
    secondObjectWidth = floor(secondObjectWidth+xSMean);
    
    
    
    checkY = intersect(height,secondObjectHeight);
    checkX = intersect(width,secondObjectWidth);
    
    if ~isempty(checkX) & ~isempty(checkY)
        estCollision = 1;
        break;
    end
    
    
end
    
   
    


