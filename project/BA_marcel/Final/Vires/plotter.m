function [ ] =        plotter(frame,flow_frame,x,flowstop,plotTime);

%Show the estimated Optical Flow 
figure(1)
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[8 8],'ScaleFactor',3);
title(sprintf('Optical Flow in iteration %.2f %',x));
drawnow;
hold off;

figure(3);
subplot(2,1,1)
plot(flowstop);
title('Time to estimate Optical Flow');
ylim([0,0.5]);
subplot(2,1,2);
plot(plotTime);
title('Time to plot');



end

