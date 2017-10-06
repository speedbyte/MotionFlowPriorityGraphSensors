% Generate Optical Flow from /training/image_2/
% Set Validation points from velocity vectors in flow2_sparse.
% Quantization for kitti color code
% Save color coded flow in results_sparse(dense)

% Read ground truth from training/flow_noc/
% Compare and find error. >5% and >3px. The error is primarily when the
% difference in the magnitude of the displacement vector between the ground
% truth and the estimated image is more than 5% of the ground truth displacement vector.
% However, for very small displacements, it is not possible to have 5%
% error difference. Hence, in the case when 5% from the magnitude of the
% ground truth displacement vector is lesser than 3px, then a cap is put on
% 3px ( not lesser ).

% Convert matrix to color code, save in RGB format and imshow

clear all;


%create Optical Flow Object for deep OF via Farneback. Uncomment to use
%opticFlow = opticalFlowFarneback%('NoiseThreshold',0.009);

%Create Optical Flow Object for sparse OF via Lucas Kanade (not as pretty
%in Kitti but still works with worse results. Uncomment to use
%opticFlow = opticalFlowLK('NoiseThreshold',0.005);


%static inout of 2 images. To be expanded for videos


path = '../../../kitti_flow_dataset/data/stereo_flow/image_02_rain/';
fileNames = dir(fullfile(path,'*.png'));

C = cell(length(fileNames),1);

for k = 1:length(fileNames)
    filename = fileNames(k).name;
    C{k} = imread([path filename]);
end



i = 1;
j = 0

mkdir('results_dense');
mkdir('results_sparse');

while i <  length(fileNames) 
    
    opticFlow_dense = opticalFlowFarneback;%('NoiseThreshold',0.009);
    opticFlow_sparse = opticalFlowLK('NoiseThreshold',0.009);
    
     
    
    img1 = C{i};
    img2 = C{i+1};

    mkdir('./results_dense');
    mkdir('./results_sparse');
    
    name_dense = sprintf('./results_dense/%06d_10.png',j);
    name_sparse = sprintf('./results_sparse/%06d_10.png',j);
    %Grayscale
    frameGray1 = rgb2gray(img1);
    frameGray2 = rgb2gray(img2);
    
    %calc the Optical Flow on Frame 1 with the used procedure
    estimateFlow(opticFlow_dense,frameGray1);
    estimateFlow(opticFlow_sparse,frameGray1);
    
    %calc OF on frame 2. This step is the important one
    flow2_dense = estimateFlow(opticFlow_dense,frameGray2);
    flow2_sparse = estimateFlow(opticFlow_sparse,frameGray2);
    
    % ----------- DENSE -----------%
    % For the Validation channel.
    vxCopy_dense = (flow2_dense.Vx ~= 0);
    vyCopy_dense = (flow2_dense.Vy ~= 0);
    vXYCopy_dense = vxCopy_dense+vyCopy_dense;
    vCopy_dense = (vXYCopy_dense ~= 0);
    
   
    %Quantize for Kitti Evaluation set
    vx_dense = (flow2_dense.Vx*64)+2^15;
    vy_dense = (flow2_dense.Vy*64)+2^15;
    
    %Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
    %and Validation bit
    res_dense = cat(3,vx_dense,vy_dense,vCopy_dense);
    re16_dense =  uint16(res_dense);
    %disp(re(:,:,1));
    
    % ----------- SPARSE -----------%
    % For the Validation channel.
    vxCopy_sparse = (flow2_sparse.Vx ~= 0);
    vyCopy_sparse = (flow2_sparse.Vy ~= 0);
    vXYCopy_sparse = vxCopy_sparse+vyCopy_sparse;
    vCopy_sparse = (vXYCopy_sparse ~= 0);
    
   
    %Quantize for Kitti Evaluation set
    vx_sparse = (flow2_sparse.Vx*64)+2^15;
    vy_sparse = (flow2_sparse.Vy*64)+2^15;
    
    %Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
    %and Validation bit
    res_sparse = cat(3,vx_sparse,vy_sparse,vCopy_sparse);
    re16_sparse =  uint16(res_sparse);
     %disp(re(:,:,1));
    
         
    %Creates OF dense estimate png for Kitti. Not very lucidly if looked at
    imwrite( re16_dense,name_dense);
    %Creates OF sparse estimate png for Kitti. Not very lucidly if looked at
    imwrite( re16_sparse,name_sparse);
    
    i = i+2;
    j = j+1
    
end    

    %This shows the Optical Flow with arrows
    %imshow(img2)
    %hold on
    %plot(flow2,'DecimationFactor',[5 5],'ScaleFactor',10)
    %hold off
    %display(flow2);


