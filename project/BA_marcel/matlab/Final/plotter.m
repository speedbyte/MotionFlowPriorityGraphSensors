function [ output_args ] = plotter(frame,flow_frame,collisionVector,estimatedCollisionVector,actualX,actualY,secondActualX,secondActualY,movement,x,timeToGenerateObject,flowstop,timeCollision,plotTime);




%Show the estimated Optical Flow 
figure(1)
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[8 8],'ScaleFactor',3);
title('Optical FLow');
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
axis([0,1242,0,375]);


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
%error, when displacement >3 px & ggf> 5%
tau = [0 0.01];
name = sprintf('./GroundTruth/%06d_10.png',x);
F_gt = flow_read(name);
F_est = flow_read('result.png');
f_err = flow_error(F_gt,F_est,tau);
F_err = flow_error_image(F_gt,F_est,tau);
figure(3);
imshow([flow_to_color([F_est;F_gt],100);F_err]);
title(sprintf('Dense Flow Error: %.2f %',f_err*100));


end

