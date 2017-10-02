function [ ] = plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,movement,x,flowstop,plotTime,collisionTime,timeMovement)




%Show the estimated Optical Flow 
figure(1)
imshow(frame);
%hold on
%plot(flow_frame,'DecimationFactor',[8 8],'ScaleFactor',3);
%title(sprintf('Optical Flow in iteration %.2f %',x));
drawnow;
hold off;


%Show the collision
figure(2)
subplot(3,1,1)
plot(collisionVector);
title('Ground Truth collision');
axis([0,length(collisionVector),0,1]);

subplot(3,1,2);
plot(estimatedCollisionVector);
title('Estimated Flow Collision');
axis([0,length(collisionVector),0,1]);


%Plot the movement of the objects.
subplot(3,1,3),
hold on;

set(gca,'ydir','reverse');
quiver(actualX,actualY,movement(1),movement(2));

set(gca,'ydir','reverse');
quiver(secondActualX,secondActualY,movement(3),movement(4));
title('Movement');
axis([0,1242,0,375]);



figure(3);
subplot(4,1,1)
plot(flowstop);
title('Time to estimate Optical Flow');
ylim([0,0.5]);
subplot(4,1,2)
plot(timeMovement);
title('Time to estimate the object movement ');
ylim([0,0.05]);
subplot(4,1,3);
plot(collisionTime);

title('Time to estimate collision');
subplot(4,1,4);
plot(plotTime);
title('Time to plot');



end

