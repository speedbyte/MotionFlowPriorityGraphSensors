function [ output_args ] = plotter( frame,flow_frame,movement,width,height,secondObjectHeight,secondObjectWidth,groundTruthCollision,estimatedCollision,negatedHeight,negatedSecondObjectHeight )

%Plot everything


fig = figure(1);
set(fig,'Position',[0,0,1242,1242]);
%Show the frame
sp1 = subplot(4,1,1);
imshow(frame);
title('Frame');


%Show the estimated Optical Flow 
sp2 = subplot(4,1,2);
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[10 10],'ScaleFactor',6);
drawnow;
title('Optical FLow');
hold off;

%Plot the movement of the objects. TODO: Use complete image scale
subplot(4,1,3),
hold on;
yyaxis left
quiver(width(floor(length(width)/2)),negatedHeight(floor(length(negatedHeight)/2)),movement(1),-movement(2));
yyaxis right
quiver(secondObjectWidth(floor(length(secondObjectWidth)/2)),negatedSecondObjectHeight(floor(length(negatedSecondObjectHeight)/2)),movement(3),-movement(4));
title('Movement');

%Collision Bool plot
subplot(4,1,4)
gT = groundTruthCollision;
est = estimatedCollision;
yyaxis left;
plot([-50,0],[gT,gT]);
yyaxis right;
plot([0,50],[est,est]);
legend('Ground Truth','Optical Flow');
title('Collision');
pause(0.7);

end

