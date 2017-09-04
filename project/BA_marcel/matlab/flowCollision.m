function [ output_args ] = flowCollision( absoluteFlow, width,height, secondObjectWidth,secondObjectHeight)

%% (use width+height+threshold) and secondWidth/height/threshold
% work with those values and check for collision then 
% At that moment only say something about unavoidable collisions. 

%TODO Threshold

collision = 0;

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
    disp('Estimated Flow collision');
end


%TODO estimate future collisions based on actual movement 