function [ ] = plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,movement,x,timeToGenerateObject,flowstop,plotTime,collisionTime,timeMovement,error,f_err,errorMean,F_est,F_gt,F_err)




%Show the estimated Optical Flow 
figure(1)
imshow(frame);
%hold on
%plot(flow_frame,'DecimationFactor',[8 8],'ScaleFactor',3);
title(sprintf('Optical Flow in iteration %.2f %',x));
%drawnow;
hold off;


%Show the collision
figure(2)
subplot(3,1,1)
plot(collisionVector);
title('Ground Truth collision');

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


fig3 = figure(3);
subplot(4,1,1);
image(flow_to_color(F_est,100));
title('Estimated Flow');
subplot(4,1,2)
image(flow_to_color(F_gt,100));
title('Ground Truth Flow');
subplot(4,1,3);
image(F_err);
title(sprintf('Flow Error: %.2f %',f_err));
set(fig3, 'Position',[0,0,1000,900]);
subplot(4,1,4);
plot(error);
axis([1,x+1,0,20]);
title(sprintf('Flow Error Mean %2f%',errorMean(x)));


figure(4);
subplot(5,1,1)
plot(timeToGenerateObject);
title('Time to generate and move Object');
ylim([0,0.1]);
subplot(5,1,2)
plot(flowstop);
title('Time to estimate Optical Flow');
ylim([0,0.5]);
subplot(5,1,3)
plot(timeMovement);
title('Time to estimate the object movement ');
ylim([0,0.05]);
subplot(5,1,4);
plot(collisionTime);

title('Time to estimate collision');
subplot(5,1,5);
plot(plotTime);
title('Time to plot');



end

