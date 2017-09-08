%This generates two objects moving somewhere in the plane, frame by
%frame(at the moment not with the infinity path).
%Relative and absolute Ground Truth is generated
%Relative and absolute Optical Flow is estimated.

%TODO is the collision algorithm and the movement along the path (which I
% dont know how to accomplish at the moment)

%%
close all;

%Initialisation of the flow object
opticFlow=opticalFlowFarneback%('NoiseThreshold',0.04);
%opticFlow=opticalFlowLK%('NoiseThreshold',0.04);

%create frame matrix
frame = zeros(375,1242,3,'uint8');
%create absolute Flow matrix
absoluteFlow = zeros(375,1242,3,'uint16');


%Create the two objects(corresponding to pedesterians)
%and their corresponding movement in x and y direction.
xMovement =3;
yMovement = 7;

width = 320:350;
height = 26:106;

secondObjectWidth = 520:550;
secondObjectHeight = 100:180;

secondXMovement = -5;
secondYMovement = 1;

%how many iterations?
maxIteration = 20;
%%
%%move the objects
for counter=1:maxIteration
    %Move the frames
    frame = Movement(height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement);
    %calculate absolute Ground Truth
    absoluteGroundTruth = gT(height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement,'absolute');
    
    if counter == 1
        figure('Name','Starting Config'),
        imshow(frame)
        pause(2)
        close;
    end
    
    %%OpticalFlow
    frame_gray = rgb2gray(frame);
    flow_frame = estimateFlow(opticFlow,frame_gray);
    
    if counter > 1 %to kick out bad first flow
        
        %get absolute estimated flow.
        for k=height
            for j=width
                absoluteFlow(k,j,1) = flow_frame.Vx(k,j)+j;
                absoluteFlow(k,j,2) = flow_frame.Vy(k,j)+k;
                absoluteFlow(k,j,3) = 1;
            end
        end
        for k=secondObjectHeight
            for j=secondObjectWidth
                absoluteFlow(k,j,1) = flow_frame.Vx(k,j)+j;
                absoluteFlow(k,j,2) = flow_frame.Vy(k,j)+k;
                absoluteFlow(k,j,3) = 1;
            end
        end
        
        
        %% collision checkers
        groundTruthCollision = gtCollision(maxIteration,absoluteGroundTruth, width,height,xMovement,yMovement, secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement,counter );
        
        %create flow matrix to store the estimated displacemend in.
        flow(:,:,1) = flow_frame.Vx;
        flow(:,:,2) = flow_frame.Vy;
        
        %estimate if a collision is about to happen.
        [estimatedCollision,movement] = flowCollision( absoluteFlow,flow, width,height, secondObjectWidth,secondObjectHeight);
        
        if groundTruthCollision == 1
            disp('Ground Truth: The objects will collide.');
        %    pause(3);
        end
        
        if estimatedCollision == 1
            disp('FLOW: Collision in the future');
        end
        
        %plot everything
        plotter(frame,flow_frame,movement,width,height,secondObjectHeight,secondObjectWidth,groundTruthCollision,estimatedCollision);
    end
    %adjust object position according to movement
    width=width+xMovement;
    height=height+yMovement;
    secondObjectHeight=secondObjectHeight+secondYMovement;
    secondObjectWidth=secondObjectWidth+secondXMovement;
    
    
    
    
    
end