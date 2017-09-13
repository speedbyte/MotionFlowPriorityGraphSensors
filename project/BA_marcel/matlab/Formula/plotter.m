function [ output_args ] = plotter( frame,flow_frame,movement,actualX,actualY,secondActualX,secondActualY,groundTruthCollision,estimatedCollision,negatedHeight,negatedSecondObjectHeight )

%Plot everything


fig = figure(1);
set(fig,'Position',[0,0,1242,1242]);
%Show the frame
sp1 = subplot(4,1,1);
imshow(frame);
title('Frame');


%Show the estimated Optical Flow 
subplot(4,1,2);
imshow(frame);
hold on
plot(flow_frame,'DecimationFactor',[5 5],'ScaleFactor',10);
drawnow;
title('Optical FLow');
hold off;

%Plot the movement of the objects.
subplot(4,1,3),
hold on;
yyaxis left
set(gca,'ydir','reverse');
quiver(actualX,actualY,movement(1),movement(2));
yyaxis left
set(gca,'ydir','reverse');
quiver(secondActualX,secondActualY,movement(3),movement(4));
title('Movement');
axis([0,1242,0,375]);

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
