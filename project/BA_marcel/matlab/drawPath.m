%Calculate image sequence.
%estimate OF on sequence
%plot movement of object
%At that moment, the object is traveling a circle

%With multiple objects, working with mean to plot wont work of course.

%But only after this small piece of work, I can say, that we will need lots
%of fps or very slow movement or otherwise, both Farnback and LK will
%definitely not work for the collision estimation (Farnback can handle
%greater displacements
%%

opticFlow=opticalFlowLK%('NoiseThreshold',0.04);

%Initialization of general movement for plotting
xMovement = 0;
yMovement = 0;

img1 = zeros(375,1242,3,'uint8');
img2 = zeros(375,1242,3,'uint8');

c = 0;
cc = 0;
m = 0;
mm = 0;
MAXVALUE = 35; %value for the iteration over the image generation
MINVALUE = -35;

figure, %creating the figure for plotting the movement.

%%
while c < MAXVALUE || cc < MAXVALUE || m > MINVALUE || mm > MINVALUE
    
    
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
    for k=250:310
        for j=220:250
            for i=1:3
                img1(k-c-m,j+cc+mm,i)= 0;
            end
        end
    end
    
    
    
    %%OpticalFlow
    im1 = rgb2gray(img1);
    flow = estimateFlow(opticFlow,im1);
    
    
    %% This shows the Optical Flow with arrows
    %     figure
    %      imshow(img1);
    %      hold on
    %      plot(flow,'DecimationFactor',[10 10],'ScaleFactor',10);
    %      hold off
    %      display(flow);
    %%
    
    %%Operations for plotting (at that moment: calculating the mean of the
    %%displacement, as there is only one object.)
    
    if c > 0 %%kicking out first flow as that one is bad
        
        x = nonzeros(flow.Vx);
        xThreshold = find(abs(x)<0.09); %cutting out very small displacements
        x(xThreshold) = [];
        
        y = nonzeros(flow.Vy);
        yThreshold = find(abs(y)<0.09);
        y(yThreshold) = [];
        
        xMean=mean(x);
        yMean=mean(y);
        yMean=-yMean; %negate y for plotting (top left of image is (0,0))
        
        disp('means');
        disp(xMean);
        disp(yMean);
        
        if isnan(xMean)
            xMean = 0;
        end
        if isnan(yMean)
            yMean = 0;
        end
        
        
        actualx = xMovement; %calculate coordinates for vector
        actualy = yMovement;
        
        xMovement = xMovement+xMean;
        yMovement = yMovement+yMean;
        
        hold on;
        quiver(actualx,actualy,xMean,yMean,0)
        drawnow;
        pause(0.1);
    end
    
    
    if c < MAXVALUE
        c= c+1;
    end
    
    if c == MAXVALUE && cc < MAXVALUE
        cc = cc+1;
    end
    
    if cc == MAXVALUE && m > MINVALUE
        m = m-1;
    end
    
    if m == MINVALUE
        mm = mm-1;
    end
    
    
end
%%









