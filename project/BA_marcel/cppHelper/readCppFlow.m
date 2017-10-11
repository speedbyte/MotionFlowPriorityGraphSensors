
for it =0:199

xFlow = sprintf('./../FlowTextFiles/FlowX/x%03d.txt',it);
yFlow = sprintf('./../FlowTextFiles/FlowY/y%03d.txt',it);
name_flow = sprintf('./../../../kitti_flow_dataset/results/LK/flow_occ/%06d_10.png',it);


x = fopen(xFlow);

formatSpec = '%i %f';
sizeA = [2,inf];
[xPos] = fscanf(x,formatSpec,sizeA);
xPos = xPos';

fclose(x);

y = fopen(yFlow);
[yPos] = fscanf(y,formatSpec,sizeA);
yPos = yPos';

fclose(y);


relativeFlow = zeros(375,1242,3,'double');

for  i=1:length(xPos)
    if xPos(i) < 1243 & xPos(i) > 1 & yPos(i) < 376 & yPos(i) > 0
    relativeFlow(yPos(i,1),xPos(i,1),1) = xPos(i,2);
    relativeFlow(yPos(i,1),xPos(i,1),2) = yPos(i,2);
    end
end

relativeFlow(:,:,3) = relativeFlow(:,:,1)~=0;
relativeFlow(:,:,3) = relativeFlow(:,:,2)~=0;

    addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));

    flow_write(relativeFlow,name_flow);
    disp(it);


end
    





        


