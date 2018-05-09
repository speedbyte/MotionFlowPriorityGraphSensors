function [ output_args ] = colorFunc( input,output )


imgs = strcat(input, '*_10.png')
    disp(imgs)
    files = dir(imgs);
    x = 1;
    
    
    
    for file = files'
        
        fileName = strcat(input, file.name);
        imgData = imread(fileName);
        flow = flow_read(fileName);
        %outPutPath = sprintf(strcat(output,'0000%02d_10.png'),x);
        outPutPath = strcat(output, file.name)
        colorFlow = flow_to_color(flow,20);
        imwrite(colorFlow, outPutPath);
        x=x+1;
    end


end

