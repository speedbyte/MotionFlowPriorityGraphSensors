
gT = './GroundTruthGenerator.m';
run(gT)

iG = './ImageGenerator.m';
run(iG)


mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ');
estFlow('slow',0);
%eval
mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_dynamic_BG');
estFlow('slow',1);
mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_static_BG');
estFlow('slow',2);
mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_static_FG');
estFlow('slow',3);
mkdir('./../../../../matlab_dataset/results/FB/slow/flow_occ_dynamic_FG');
estFlow('slow',4);



mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ');
estFlow('normal',0);
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_dynamic_BG');
estFlow('normal',1);
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_static_BG');
estFlow('normal',2);
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_static_FG');
estFlow('normal',3);
mkdir('./../../../../matlab_dataset/results/FB/normal/flow_occ_dynamic_FG');
estFlow('normal',4);


%% Experiments for fast movement < 46px
%mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ');
%estFlow('fast',0);

mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ');
estFlow('fast',0);
mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_dynamic_BG');
estFlow('fast',1);
mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_static_BG');
estFlow('fast',2);
mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_static_FG');
estFlow('fast',3);
mkdir('./../../../../matlab_dataset/results/FB/fast/flow_occ_dynamic_FG');
estFlow('fast',4);









