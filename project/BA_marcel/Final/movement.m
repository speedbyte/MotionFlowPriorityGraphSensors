function [ frame ] = movement(xSpec,ySpec,secondXSpec,secondYSpec,bg )
%MOVEMENT Summary of this function goes here
%   Detailed explanation goes here
frame = zeros(375,1242,3,'uint8');
r = 0;
b = 0;

   %%
    %reset the image to white
    for k=1:375
        for j=1:1242
                frame(k,j,1) = 255;
                frame(k,j,2) = 255;
                frame(k,j,3) = 255; 
                
        end
    end

        
    
    %draw new image.
    for k = ySpec
        for j= xSpec
             frame(k,j,1)= r;
             frame(k,j,2)= b;
             frame(k,j,3)= 0;
             r=r+2;
             b=b+2;
             if r > 254
                 r = 130;
             end
             if b > 254;
                 b = 46;
             end
        end
    end
        
r = 0;
b = 0;
    
    %expand with 2nd Object
    for k = secondYSpec
        for j= secondXSpec
            frame(k,j,1)= r;
             frame(k,j,2)= b;
             frame(k,j,3)= 0;
             r=r+2;
             b=b+2;
             if r > 254
                 r = 130;
             end
             if b > 254;
                 b = 46;
             end
        end
    end
    
%     %static noise
%     for k=1:375
%         for j=1:1242
%             if mod(k,5) == 0 | mod(j,5) == 0
%                 frame(k,j,1) = 0;
%                 frame(k,j,2) = 0;
%                 frame(k,j,3) = 0;
%             end
%         end
%     end
%     
%    
%  

    
