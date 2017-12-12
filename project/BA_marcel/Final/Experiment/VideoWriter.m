
clear all;
close all;

%Create Video out of frames.
writer = VideoWriter('Movement.avi');
open(writer);
for i = 1:maxIteration
    name  = sprintf('./Videos/Frames/%06d.png',i);
    img = imread(name);
    writeVideo(writer,img);
end
close(writer);

close all;