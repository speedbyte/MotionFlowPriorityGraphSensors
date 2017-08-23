function [] = gT( height,width,xMovement,yMovement )
%%generate GT

img3 = zeros(375,1242,3,'uint16');

for k=height
    for j=width
        img3(k,j,1) = xMovement;
        img3(k,j,2) = yMovement;
        img3(k,j,3) = 1;
    end
end

gtX = (img3(:,:,1)*64)+2^15;
gtY = (img3(:,:,2)*64)+2^15;

%Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
%and Validation bit
res = cat(3,gtX,gtY,img3(:,:,3));
re =  uint16(res);
%disp(re(:,:,1));

%Creates OF png for Kitti. Not very lucidly if looked at
imwrite( re,'GroundTruth.png');

disp('GroundTruth has been generated')

end

