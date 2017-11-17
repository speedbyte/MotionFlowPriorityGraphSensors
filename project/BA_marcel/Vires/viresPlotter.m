function [ output_args ] = viresPlotter( img_gt,counter,x,color )
fig3 = figure(1);
subplot(3,1,1);
image(img_gt);
title(sprintf('Visualized Flow in Iteration %i', x+1));
subplot(3,1,2)
plot(counter)
title(sprintf('Correct Pixels in Iteration %i: %i',x+1,counter(x+1)));
subplot(3,1,3);
image(color);
title(sprintf('Complete Flow in Iteration %i', x+1));
drawnow;




end

