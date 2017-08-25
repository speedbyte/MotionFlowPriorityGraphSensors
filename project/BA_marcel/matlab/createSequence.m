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

xMovement = 10;
yMovement = 10;

width = 200:260;
height = 46:106;


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

figure('Name', 'First Frame');
imshow(img1);

%%create frame 2
for k=height
    for j=width
        for i=1:3
            img2(k+yMovement,j+xMovement,i)= 0;
            if mod(k,25) == 0
                img2(k,j,i) = 255;
            end
            if mod(j,25) == 0
                img2(k,j,i) = 255;
            end
        end
    end
end

figure('Name', 'Second Frame')
imshow(img2);

%calculate Ground Truth%
gT(height,width,xMovement,yMovement);


%%OpticalFlow
img1_gray = rgb2gray(img1);
flow_img1 = estimateFlow(opticFlow,img1_gray);

img2_gray = rgb2gray(img2);
flow_img2 = estimateFlow(opticFlow,img2_gray)


%% This shows the Optical Flow with arrows
figure('Name', 'Displacement Vector')
imshow(img1);
hold on
plot(flow_img1,'DecimationFactor',[5 5],'ScaleFactor',1);
hold off
display(flow_img1);
%%

%Threshold
flowX = flow_img2.Vx;
flowy = flow_img2.Vy;
indices = find(abs(flow_img2.Vx)<0.001);
flowX(indices) = 0;

indices = find(abs(flow_img2.Vy)<0.001);
flowy(indices) = 0;

% For the Validation channel.
vxCopy = (flowX ~= 0);
vyCopy = (flowy ~= 0);
vXYCopy = vxCopy+vyCopy;
vCopy = (vXYCopy ~= 0);


%Quantize for Kitti Evaluation set
vx = (flow_img2.Vx*64)+2^15;
vy = (flow_img2.Vy*64)+2^15;

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
res = cat(3,vx,vy,vCopy);
re =  uint16(res);
%disp(re(:,:,1));

%Creates OF png for Kitti. Not very lucidly if looked at
imwrite( re,'created.png');


