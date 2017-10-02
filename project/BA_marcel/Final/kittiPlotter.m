function [ output_args ] = kittiPlotter( error,f_err,errorMean,F_est,F_gt,F_err,x )
fig3 = figure(1);
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
drawnow;

end

