function [ output_args ] = Untitled( name,noise,folder )

load(name);

%%Generates the frames




for x=1:maxIteration+1
    
    %Used to store the GT images for the kitti devkit
  name_frame = sprintf('./../../../../matlab_dataset/data/stereo_flow/image_02_%s/%06d_10.png',folder,x-2);

    %Initialization
    if x == 1
    iterator = 0;
    sIterator = 0;
    end
    
    %%
    %Ground Truth Movement (First Object x, first Object y, second object
    %x, second object y movement
    [calcMove] = gtMovement(xPos,yPos,start,secondStart,iterator,sIterator,x);
    
    %If we are at the end of the path vector, we need to reset our
    %iterators
     if iterator+start > length(xPos)
        start = 1;
        iterator = 0;
    end

    if sIterator+secondStart > length(xPos)
        secondStart = 1;
        sIterator = 0;
    end
    
  
    %Object specification
    xSpec = actualX:actualX+width;  %width
    ySpec = actualY:actualY+height; %height
    secondXSpec = secondActualX:secondActualX+width;
    secondYSpec = secondActualY:secondActualY+height;
    
        frame = movement(xSpec,ySpec,secondXSpec,secondYSpec,noise);
        if x > 1
        imwrite(frame,name_frame);
        end

%%    

    
    iterator = iterator+1;
    sIterator = sIterator+1;
       
    actualX = actualX+calcMove(1);
    actualY = actualY+calcMove(2);
    secondActualX = secondActualX+calcMove(3);
    secondActualY = secondActualY+calcMove(4);

    disp(x);


    
end

end

