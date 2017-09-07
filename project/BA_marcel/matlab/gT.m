function [img4] = gT( height,width,xMovement,yMovement,secondObjectWidth,secondObjectHeight,secondXMovement,secondYMovement )
%%generate GT

img3 = zeros(375,1242,3,'uint16');
img4 = zeros(375,1242,3);

for k=height
    for j=width
        img3(k,j,1) = xMovement;
        img3(k,j,2) = yMovement;
        img3(k,j,3) = 1;
        img4(k,j,1) = xMovement+j;
        img4(k,j,2) = yMovement+k;
        img4(k,j,3) = 1;
    end
end

for k=secondObjectHeight
    for j=secondObjectWidth
        img3(k,j,1) = secondXMovement;
        img3(k,j,2) = secondYMovement;
        img3(k,j,3) = 1;
        img4(k,j,1) = secondXMovement+j;
        img4(k,j,2) = secondYMovement+k;
        img4(k,j,3) = 1;
    end
end

%%Quantization
gtX = (img3(:,:,1)*64)+2^15;
gtY = (img3(:,:,2)*64)+2^15;

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
res = cat(3,img3(:,:,1),img3(:,:,2),img3(:,:,3));
re =  uint16(res);
%disp(re(:,:,1));

%Creates OF png for Kitti. Not very lucidly if looked at

%Comment in if needed
%imwrite( re,'GroundTruth.png');
%disp('GroundTruth has been generated')

end

