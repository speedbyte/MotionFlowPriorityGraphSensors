%Loads the Initialization specs. 



clear all;
close all;

opticFlow=opticalFlowFarneback%('NoiseThreshold',0.004);





plotTime = 1;
error(1) = 0;
error(2) = 0;




for x = 1:200
    
    disp(x);

    name_frame = sprintf('./../../../../vires_dataset/data/stereo_flow/image_02/%07d.png',x);   %imread
    name_flow = sprintf('./../../../../vires_dataset/results/FB/flow_occ/%06d_10.png',x);          %result

    


    
    %%
    %Optical Flow
   tic;
   frame = imread(name_frame);
   frame_gray = rgb2gray(frame);
   
   flow_frame = estimateFlow(opticFlow,frame_gray);
   flowstop(x) = toc;
  
   
    vxCopy = (flow_frame.Vx ~= 0);
    vyCopy = (flow_frame.Vy ~= 0);
    vXYCopy = vxCopy+vyCopy;
    vCopy = (vXYCopy ~= 0);
    
     res = cat(3,flow_frame.Vx,flow_frame.Vy,vCopy);
     
     addpath(genpath('../../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
     
     flow_write(res,name_flow);
     

         
      %create flow matrix to store the estimated displacemend in.
        flow(:,:,1) = flow_frame.Vx;
        flow(:,:,2) = flow_frame.Vy;
        
      %absolute Flow
      %get absolute estimated flow.
              
        
        %%
        %%collision checkers(first round has bad flow, dont plot and
        %%estimate collision
        %time to check for collision
        
        
       %  estimatedCollision = flowCollision(absoluteFlow, xSpec,ySpec, secondXSpec,secondYSpec);
        

       tic;
        
       plotter(frame,flow_frame,x,flowstop,plotTime);
       
       plotTime(x) = toc;

   
    
end
