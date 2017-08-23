%Creates white Image with black rectangle
%Create second image with moved rectangle
%%Calculate OF
%Plot movement

%Next step will be:
%Moving the frames in a for loop and iteratively expand the plot
%With multiple objects, working with mean to plot wont work of course.

%But only after this small piece of work, I can say, that we will need lots
%of fps or very slow movement or otherwise, both Farnback and LK will
%definitely not work!




img1 = zeros(375,1242,3,'uint8');
img2 = zeros(375,1242,3,'uint8');

%%white background
for k=1:375
    for j=1:1242
        for i=1:3
            img1(k,j,i) = 255;
            img2(k,j,i) = 255;
        end
    end
end
%%

%%create frame1
for k=250:310
    for j=220:250
        for i=1:3
            img1(k,j,i)= 0;
        end
    end
end

%create frame2
for k=220:280
    for j=230:260
        for i=1:3
            img2(k,j,i)= 0;
        end
    end
end
%%

    
imshow(img1);

%%OpticalFlow
opticFlow=opticalFlowLK%('NoiseThreshold',0.04);
im1 = rgb2gray(img1);
im2 = rgb2gray(img2);
flow = estimateFlow(opticFlow,im1);
flow2 = estimateFlow(opticFlow,im2);
%%

%% This shows the Optical Flow with arrows
    imshow(img1)
    hold on
    plot(flow2,'DecimationFactor',[10 10],'ScaleFactor',1)
    hold off
    display(flow2);
    
    figure,
    imshow(img2);
 %%   
 
 %%Operations for plotting
   x = nonzeros(flow2.Vx);
   xThreshold = find(abs(x)<0.09); %cutting out very small displacements
   x(xThreshold) = [];
       
   y = nonzeros(flow2.Vy);
   yThreshold = find(abs(y)<0.09);
   y(yThreshold) = [];
   
   xMean=mean(x);
   yMean=mean(y);
   yMean=-yMean; %negate y for plotting (top of image is (0,0))
   
   disp('means');
   disp(xMean);
   disp(yMean);
   
   %plot
   figure
   plot([0,xMean],[0,yMean]);
   axis equal;
   

 

    


    
    