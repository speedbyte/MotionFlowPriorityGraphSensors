function [ output_args ] = plotter( frame,flow_frame )

%Plot everything


figure(1);
 subplot(2,1,1),
imshow(frame);

subplot(2,1,2),
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[10 10],'ScaleFactor',6);
drawnow;

end

