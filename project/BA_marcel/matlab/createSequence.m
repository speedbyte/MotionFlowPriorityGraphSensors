%Creates an image sequence. Estimates Optical Flow of it and calculates
%Ground Truth. Both in Kitti Format

%But only after this small piece of work, I can say, that we will need lots
%of fps or very slow movement or otherwise, both Farnback and LK will
%definitely not work for the collision estimation (Farnback can handle
%greater displacements
%%
close all;

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.04);

img1 = zeros(375,1242,3,'uint8');
img2 = zeros(375,1242,3,'uint8');
absoluteFlow = zeros(375,1242,3,'uint16');

xMovement = 3;
yMovement = 0;

width = 200:260;
height = 46:106;

secondObjectWidth = 599:659;
secondObjectHeight = 46:106;

secondXMovement = -3;
secondYMovement = 0;


%%

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



%%create frame 2
for k=height
    for j=width
        for i=1:3
            img2(k+yMovement,j+xMovement,i)= 0;
            if mod(k,25) == 0
                img2(k+yMovement,j,i) = 255;
            end
            if mod(j,25) == 0
                img2(k,j+xMovement,i) = 255;
            end
        end
    end
end

%%expand with 2nd object
for k = secondObjectHeight
    for j= secondObjectWidth
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


%%create frame 2
for k=secondObjectHeight
    for j=secondObjectWidth
        for i=1:3
            img2(k+secondYMovement,j+secondXMovement,i)= 0;
            if mod(k,25) == 0
                img2(k+secondYMovement,j,i) = 255;
            end
            if mod(j,25) == 0
                img2(k,j+secondXMovement,i) = 255;
            end
        end
    end
end

figure('Name', 'First Frame');
imshow(img1);
figure('Name', 'Second Frame')
imshow(img2);

%calculate absolute Ground Truth and create Kitti png for relative Ground Truth%
absoluteGT = gT(height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement);


%%OpticalFlow
img1_gray = rgb2gray(img1);
flow_img1 = estimateFlow(opticFlow,img1_gray);

img2_gray = rgb2gray(img2);
flow_img2 = estimateFlow(opticFlow,img2_gray)


%% This shows the Optical Flow with arrows
figure('Name', 'Displacement Vector')
imshow(img1);
hold on
plot(flow_img2,'DecimationFactor',[6 6],'ScaleFactor',2);
hold off
display(flow_img2);
%%

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


%Threshold
flowX = flow_img2.Vx;
flowY = flow_img2.Vy;
indices = find(abs(flow_img2.Vx)<0.001);
flowX(indices) = 0;

indices = find(abs(flow_img2.Vy)<0.001);
flowY(indices) = 0;

% For the Validation channel.
vxCopy = (flowX ~= 0);
vyCopy = (flowY ~= 0);
vXYCopy = vxCopy+vyCopy;
vCopy = (vXYCopy ~= 0);


%Quantize for Kitti Evaluation set
%vx = (flow_img2.Vx*64)+2^15;
%vy = (flow_img2.Vy*64)+2^15;

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
res = cat(3,flowX,flowY,vCopy);
re =  uint16(res);
%disp(re(:,:,1));

%Creates OF png for Kitti. Not very lucidly if looked at
imwrite( re,'estimatedFlow.png');


