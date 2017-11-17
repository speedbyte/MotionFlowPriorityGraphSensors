
mkdir('./../../../../matlab_dataset/data/stereo_flow/flow_occ_slow');
mkdir('./../../../../matlab_dataset/data/stereo_flow/flow_occ_normal');
mkdir('./../../../../matlab_dataset/data/stereo_flow/flow_occ_fast');



%Experiment Init

%slow
%speed = 0.5;
%slow = init(speed,1);
%gtCollision(slow,'slow',0);





%normal
%speed = 2;
%normal = init(speed,2);
%gtCollision(normal,'normal',1);


%fast
speed = 6;
fast = init(speed,3);
gtCollision(fast,'fast',2);




