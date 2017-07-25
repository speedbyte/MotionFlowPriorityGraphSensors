clear all;


%create Optical Flow Object for deep OF via Farneback. Uncomment to use
%opticFlow = opticalFlowFarneback%('NoiseThreshold',0.009);

%Create Optical Flow Object for sparse OF via Lucas Kanade (not as pretty
%in Kitti but still works with worse results. Uncomment to use
opticFlow = opticalFlowLK('NoiseThreshold',0.005);


%static inout of 2 images. To be expanded for videos
img1 = imread('000016_10.png');
img2 = imread('000016_11.png');

    %Grayscale
    frameGray1 = rgb2gray(img1);
    frameGray2 = rgb2gray(img2);
    
    %calc the Optical Flow on Frame 1 with the used procedure
    estimateFlow(opticFlow,frameGray1);
    
    
    %calc OF on frame 2. This step is the important one
    flow2 = estimateFlow(opticFlow,frameGray2);
    
    % For the Validation channel.
    vxCopy = (flow2.Vx ~= 0);
    vyCopy = (flow2.Vy ~= 0);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
   
   %Quantize for Kitti Evaluation set
   vx = (flow2.Vx*64)+2^15;
   vy = (flow2.Vy*64)+2^15;
    
   %Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
   %and Validation bit
    res = cat(3,vx,vy,vCopy);
    re =  uint16(res);
     %disp(re(:,:,1));
    
    %Creates OF png for Kitti. Not very lucidly if looked at
    imwrite( re,'result16.png');
    im = imread('result6.png');
    %disp(im(:,:,1));
    

    %This shows the Optical Flow with arrows
    imshow(img2)
    hold on
    plot(flow2,'DecimationFactor',[5 5],'ScaleFactor',10)
    hold off
    display(flow2);


