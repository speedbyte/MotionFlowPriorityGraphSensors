%This generates two objects moving somewhere in the plane, frame by
%frame(at the moment not with the infinity path). 
%Relative and absolute Ground Truth is generated
%Relative and absolute Optical Flow is estimated.

%TODO is the collision algorithm and the movement along the path (which I
% dont know how to accomplish at the moment)

%%
close all;

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.04);

img1 = zeros(375,1242,3,'uint8');
img2 = zeros(375,1242,3,'uint8');
absoluteFlow = zeros(375,1242,3,'uint16');

xMovement = 4;
yMovement = 1;

width = 320:380;
height = 46:106;

secondObjectWidth = 520:580;
secondObjectHeight = 5:65;

secondXMovement = -5;
secondYMovement = 1;

%%
%%Initialize
%%white background
for k=1:375
    for j=1:1242
        for i=1:3
            img1(k,j,i) = 255;
            img2(k,j,i) = 255;
            
        end
    end
end

 
%%create frame
for k = height
    for j= width
        for i=1:3
            img1(k,j,i)= 0;
            if mod(k,25) == 0
                img1(k,j,i) = 255;
            end
            if mod(j,25) == 0
                img1(k,j,i) = 255;
            end
        end
    end
end


%%expand with 2nd object
for k = secondObjectHeight
    for j= secondObjectWidth
        for i=1:3
            img1(k,j,i)= 0;
            if mod(k,5) == 0
                img1(k,j,i) = 255;
            end
            if mod(j,5) == 0
                img1(k,j,i) = 255;
            end
        end
    end
end
figure('Name', 'Start');
imshow(img1);
pause(1);
close;
%%


%%move the objects
for counter=1:20

  img1 = Movement(height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement);

%%



figure('Name', 'First Frame');
imshow(img1);

pause(0.5);
close all;
%%
%calculate absolute Ground Truth and create Kitti png for relative Ground Truth%
absoluteGT = gT(height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement);


%%OpticalFlow
img1_gray = rgb2gray(img1);
flow_img1 = estimateFlow(opticFlow,img1_gray);

%%get absolute estimated flow. At the moment hacked, because I only watch
%%for entries, where the objects are. If the flow detects movement beyond the object, then
%%it is capped in the absolute flow. 

for k=height
    for j=width
        absoluteFlow(k,j,1) = flow_img1.Vx(k,j)+j;
        absoluteFlow(k,j,2) = flow_img1.Vy(k,j)+k;
        absoluteFlow(k,j,3) = 1;
    end
end

for k=secondObjectHeight
    for j=secondObjectWidth
        absoluteFlow(k,j,1) = flow_img1.Vx(k,j)+j;
        absoluteFlow(k,j,2) = flow_img1.Vy(k,j)+k;
        absoluteFlow(k,j,3) = 1;
    end
end


%% This shows the Optical Flow with arrows
figure('Name', 'Displacement Vector');
imshow(img1);
hold on
plot(flow_img1,'DecimationFactor',[6 6],'ScaleFactor',13);


%% collision checkers
gtCol = gtCollision( absoluteGT, width,height,xMovement,yMovement, secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement );

flow(:,:,1) = flow_img1.Vx;
flow(:,:,2) = flow_img1.Vy;

estCol = flowCollision( absoluteFlow,flow, width,height, secondObjectWidth,secondObjectHeight);

if gtCol == 1
    disp('GT: Collision in the future');
end
if estCol == 1
    disp('FLOW: Collision in the future');
end
%%
    
%adjust object position according to movement
width=width+xMovement;
height=height+yMovement;
secondObjectHeight=secondObjectHeight+secondYMovement;
secondObjectWidth=secondObjectWidth+secondXMovement;


end