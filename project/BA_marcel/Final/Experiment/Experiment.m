gt = 0;
ig = 0;

if gt == 1
gT = './GroundTruthGenerator.m';
run(gT)
end

if ig == 1
iG = './ImageGenerator.m';
run(iG)
end


mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ');
%estFlow('slow',0);
%evaluation('slow',0);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_dynamic_BG');
%estFlow('slow',1);
%evaluation('slow',1);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_static_BG');
%estFlow('slow',2);
%evaluation('slow',2);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_static_FG');
%estFlow('slow',3);
%evaluation('slow',3);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_dynamic_FG');
%estFlow('slow',4);
%evaluation('slow',4);
disp('Press any key to start normal Experiment');
w = waitforbuttonpress;
close all;



mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ');
estFlow('normal',0);
evaluation('normal',0);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_dynamic_BG');
estFlow('normal',1);
evaluation('normal',1);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_static_BG');
estFlow('normal',2);
evaluation('normal',2);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_static_FG');
estFlow('normal',3);
evaluation('normal',3);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_dynamic_FG');
estFlow('normal',4);
evaluation('normal',4);

disp('Press any key to start fast Experiment');
w = waitforbuttonpress;
close all;


mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ');
%estFlow('fast',0);
evaluation('fast',0);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_dynamic_BG');
estFlow('fast',1);
evaluation('fast',1);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_static_BG');
estFlow('fast',2);
evaluation('fast',2);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_static_FG');
estFlow('fast',3);
evaluation('fast',3);
disp('Press any key to continue');
w = waitforbuttonpress;
close all;

mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_dynamic_FG');
estFlow('fast',4);
evaluation('fast',4);


disp('--------FINISHED---------');





