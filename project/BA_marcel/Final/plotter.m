function [ output_args ] = plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,movement,x,timeToGenerateObject,flowstop,timeCollision,plotTime,timeMovement)




%Show the estimated Optical Flow 
figure(1)
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[8 8],'ScaleFactor',3);
title(sprintf('Optical Flow in iteration %.2f %',x));
drawnow;
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
yyaxis left
set(gca,'ydir','reverse');
quiver(actualX,actualY,movement(1),movement(2));
yyaxis left
set(gca,'ydir','reverse');
quiver(secondActualX,secondActualY,movement(3),movement(4));
title('Movement');
axis([0,1242,0,375]);

%kitti plot
tau = [3 0.05];
name = sprintf('./GroundTruth/%06d_10.png',x);

addpath(genpath('./Kitti'));
F_gt = flow_read(name);
F_est = flow_read('result.png');
f_err = flow_error(F_gt,F_est,tau);
F_err = flow_error_image(F_gt,F_est,tau);
fig3 = figure(3);
subplot(3,1,1);
image(flow_to_color(F_est,100));
title('Estimated Flow');
subplot(3,1,2)
image(flow_to_color(F_gt,100));
title('Ground Truth Flow');
subplot(3,1,3);
image(F_err);
title(sprintf('Flow Error: %.2f %',f_err*100));
set(fig3, 'Position',[0,0,1000,900]);

figure(4);
subplot(5,1,1)
plot(timeToGenerateObject);
title('Time to generate and move Object');
subplot(5,1,2)
plot(flowstop);
title('Time to estimate Optical Flow');
subplot(5,1,3);
plot(timeCollision);
title('Time to estimate collision');
subplot(5,1,4)
plot(timeMovement);
title('Time to estimate the object movement');
subplot(5,1,5);
plot(plotTime);
title('Time to plot');



end

